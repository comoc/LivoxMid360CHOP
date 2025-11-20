#include "LivoxDevice.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <sstream>
#include <system_error>

namespace
{
	constexpr float kMilliToMeters = 0.001f;
	constexpr float kCentiToMeters = 0.01f;
}

LivoxDevice::LivoxDevice()
	: buffer_limit_(200000)
	, running_(false)
	, connected_(false)
	, sdk_initialized_(false)
	, lidar_handle_(0)
	, total_points_(0)
	, requested_data_type_(kLivoxLidarCartesianCoordinateHighData)
	, current_data_type_(kLivoxLidarCartesianCoordinateHighData)
{
	status_text_ = "Idle";
}

LivoxDevice::~LivoxDevice()
{
	stop();
}

bool
LivoxDevice::start(const std::string& config_path)
{
	namespace fs = std::filesystem;
	std::error_code ec;
	const fs::path cfg_path(config_path);
	if (!fs::exists(cfg_path, ec))
	{
		publishStatus("Config file not found: " + config_path);
		return false;
	}

	if (!LivoxLidarSdkInit(config_path.c_str()))
	{
		publishStatus("LivoxLidarSdkInit failed");
		return false;
	}

	clear();
	total_points_.store(0);

	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		config_path_ = config_path;
		running_ = true;
		sdk_initialized_ = true;
		connected_ = false;
		lidar_handle_ = 0;
		serial_number_.clear();
		lidar_ip_.clear();
		status_text_ = "SDK initialized, waiting for Mid-360";
	}

	SetLivoxLidarPointCloudCallBack(PointCloudCallback, this);
	SetLivoxLidarInfoCallback(InfoCallback, this);
	SetLivoxLidarInfoChangeCallback(InfoChangeCallback, this);

	return true;
}

void
LivoxDevice::stop()
{
	bool should_uninit = false;
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		if (!running_)
		{
			return;
		}
		running_ = false;
		connected_ = false;
		lidar_handle_ = 0;
		serial_number_.clear();
		lidar_ip_.clear();
		status_text_ = "Stopped";
		should_uninit = sdk_initialized_;
		sdk_initialized_ = false;
	}

	if (should_uninit)
	{
		LivoxLidarSdkUninit();
	}
}

void
LivoxDevice::clear()
{
	std::lock_guard<std::mutex> lock(buffer_mutex_);
	buffer_.clear();
}

bool
LivoxDevice::isRunning() const
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	return running_;
}

bool
LivoxDevice::isConnected() const
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	return connected_;
}

void
LivoxDevice::setBufferLimit(size_t limit)
{
	if (limit == 0)
	{
		limit = 1;
	}
	std::lock_guard<std::mutex> lock(buffer_mutex_);
	buffer_limit_ = limit;
	while (buffer_.size() > buffer_limit_)
	{
		buffer_.pop_front();
	}
}

size_t
LivoxDevice::bufferLimit() const
{
	std::lock_guard<std::mutex> lock(buffer_mutex_);
	return buffer_limit_;
}

void
LivoxDevice::setPointDataType(LivoxLidarPointDataType type)
{
	uint32_t handle = 0;
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		if (requested_data_type_ == type)
		{
			return;
		}
		requested_data_type_ = type;
		if (connected_ && lidar_handle_ != 0)
		{
			handle = lidar_handle_;
		}
	}

	if (handle != 0)
	{
		applyPendingDataType(handle);
	}
}

LivoxLidarPointDataType
LivoxDevice::requestedDataType() const
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	return requested_data_type_;
}

LivoxLidarPointDataType
LivoxDevice::activeDataType() const
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	return current_data_type_;
}

size_t
LivoxDevice::consume(PointSample* destination, size_t max_points)
{
	if (destination == nullptr || max_points == 0)
	{
		return 0;
	}

	std::lock_guard<std::mutex> lock(buffer_mutex_);
	const size_t available = std::min(max_points, buffer_.size());
	for (size_t i = 0; i < available; ++i)
	{
		destination[i] = buffer_.front();
		buffer_.pop_front();
	}
	return available;
}

size_t
LivoxDevice::bufferedSamples() const
{
	std::lock_guard<std::mutex> lock(buffer_mutex_);
	return buffer_.size();
}

std::string
LivoxDevice::statusText() const
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	return status_text_;
}

std::string
LivoxDevice::infoMessage() const
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	return info_text_;
}

std::string
LivoxDevice::lidarSerial() const
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	return serial_number_;
}

std::string
LivoxDevice::lidarIp() const
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	return lidar_ip_;
}

uint64_t
LivoxDevice::totalPoints() const
{
	return total_points_.load();
}

void
LivoxDevice::PointCloudCallback(uint32_t, const uint8_t, LivoxLidarEthernetPacket* data, void* client_data)
{
	if (client_data == nullptr || data == nullptr)
	{
		return;
	}
	auto* self = static_cast<LivoxDevice*>(client_data);
	self->handlePointCloud(data);
}

void
LivoxDevice::InfoCallback(uint32_t, const uint8_t, const char* info, void* client_data)
{
	if (client_data == nullptr || info == nullptr)
	{
		return;
	}
	auto* self = static_cast<LivoxDevice*>(client_data);
	self->handleInfoMessage(info);
}

void
LivoxDevice::InfoChangeCallback(uint32_t handle, const LivoxLidarInfo* info, void* client_data)
{
	if (client_data == nullptr || info == nullptr)
	{
		return;
	}
	auto* self = static_cast<LivoxDevice*>(client_data);
	self->handleInfoChange(handle, info);
}

void
LivoxDevice::WorkModeCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse* response, void* client_data)
{
	if (client_data == nullptr)
	{
		return;
	}
	auto* self = static_cast<LivoxDevice*>(client_data);
	std::ostringstream oss;
	if (status == kLivoxLidarStatusSuccess && response != nullptr && response->ret_code == 0)
	{
		oss << "Work mode set OK for handle " << handle;
	}
	else
	{
		oss << "Work mode failed (" << status << ")";
		if (response != nullptr)
		{
			oss << " ret=" << static_cast<int>(response->ret_code);
		}
	}
	self->publishStatus(oss.str());
}

void
LivoxDevice::DataTypeCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse* response, void* client_data)
{
	if (client_data == nullptr)
	{
		return;
	}
	auto* self = static_cast<LivoxDevice*>(client_data);
	std::ostringstream oss;
	if (status == kLivoxLidarStatusSuccess && response != nullptr && response->ret_code == 0)
	{
		oss << "Data type updated for handle " << handle;
	}
	else
	{
		oss << "Data type update failed (" << status << ")";
		if (response != nullptr)
		{
			oss << " ret=" << static_cast<int>(response->ret_code);
		}
	}
	self->publishStatus(oss.str());
}

void
LivoxDevice::handlePointCloud(LivoxLidarEthernetPacket* packet)
{
	if (packet == nullptr)
	{
		return;
	}

	const LivoxLidarPointDataType data_type = static_cast<LivoxLidarPointDataType>(packet->data_type);
	if (data_type != kLivoxLidarCartesianCoordinateHighData && data_type != kLivoxLidarCartesianCoordinateLowData)
	{
		return;
	}

	uint64_t timestamp = 0;
	std::memcpy(&timestamp, packet->timestamp, sizeof(uint64_t));

	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		connected_ = true;
		current_data_type_ = data_type;
	}

	std::lock_guard<std::mutex> lock(buffer_mutex_);
	const uint32_t dot_count = packet->dot_num;
	if (data_type == kLivoxLidarCartesianCoordinateHighData)
	{
		const auto* points = reinterpret_cast<LivoxLidarCartesianHighRawPoint*>(packet->data);
		for (uint32_t i = 0; i < dot_count; ++i)
		{
			PointSample sample;
			sample.x = static_cast<float>(points[i].x) * kMilliToMeters;
			sample.y = static_cast<float>(points[i].y) * kMilliToMeters;
			sample.z = static_cast<float>(points[i].z) * kMilliToMeters;
			sample.intensity = static_cast<float>(points[i].reflectivity);
			sample.tag = static_cast<float>(points[i].tag);
			sample.timestamp = timestamp;
			buffer_.push_back(sample);
		}
		total_points_.fetch_add(dot_count);
	}
	else
	{
		const auto* points = reinterpret_cast<LivoxLidarCartesianLowRawPoint*>(packet->data);
		for (uint32_t i = 0; i < dot_count; ++i)
		{
			PointSample sample;
			sample.x = static_cast<float>(points[i].x) * kCentiToMeters;
			sample.y = static_cast<float>(points[i].y) * kCentiToMeters;
			sample.z = static_cast<float>(points[i].z) * kCentiToMeters;
			sample.intensity = static_cast<float>(points[i].reflectivity);
			sample.tag = static_cast<float>(points[i].tag);
			sample.timestamp = timestamp;
			buffer_.push_back(sample);
		}
		total_points_.fetch_add(dot_count);
	}

	while (buffer_.size() > buffer_limit_)
	{
		buffer_.pop_front();
	}
}

void
LivoxDevice::handleInfoChange(uint32_t handle, const LivoxLidarInfo* info)
{
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		connected_ = true;
		lidar_handle_ = handle;
		serial_number_ = info->sn;
		lidar_ip_ = info->lidar_ip;
		status_text_ = "Connected to " + serial_number_ + " (" + lidar_ip_ + ")";
	}

	SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, this);
	applyPendingDataType(handle);
}

void
LivoxDevice::handleInfoMessage(const std::string& message)
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	info_text_ = message;
}

void
LivoxDevice::publishStatus(const std::string& text)
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	status_text_ = text;
}

void
LivoxDevice::applyPendingDataType(uint32_t handle)
{
	const LivoxLidarPointDataType data_type = requestedDataType();
	const livox_status status = SetLivoxLidarPclDataType(handle, data_type, DataTypeCallback, this);
	if (status != kLivoxLidarStatusSuccess)
	{
		std::ostringstream oss;
		oss << "Set data type failed (" << status << ")";
		publishStatus(oss.str());
	}
}
