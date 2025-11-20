#pragma once

#include <atomic>
#include <cstdint>
#include <cstddef>
#include <deque>
#include <mutex>
#include <string>

#include "livox_lidar_api.h"

class LivoxDevice
{
public:
	struct PointSample
	{
		float x = 0.0f;
		float y = 0.0f;
		float z = 0.0f;
		float intensity = 0.0f;
		float tag = 0.0f;
		uint64_t timestamp = 0;
	};

	LivoxDevice();
	~LivoxDevice();

	bool start(const std::string& config_path);
	void stop();
	void clear();
	bool isRunning() const;
	bool isConnected() const;

	void setBufferLimit(size_t limit);
	size_t bufferLimit() const;

	void setPointDataType(LivoxLidarPointDataType type);
	LivoxLidarPointDataType requestedDataType() const;
	LivoxLidarPointDataType activeDataType() const;

	size_t consume(PointSample* destination, size_t max_points);
	size_t bufferedSamples() const;

	std::string statusText() const;
	std::string infoMessage() const;
	std::string lidarSerial() const;
	std::string lidarIp() const;

	uint64_t totalPoints() const;

private:
	static void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data);
	static void InfoCallback(uint32_t handle, const uint8_t dev_type, const char* info, void* client_data);
	static void InfoChangeCallback(uint32_t handle, const LivoxLidarInfo* info, void* client_data);
	static void WorkModeCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse* response, void* client_data);
	static void DataTypeCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse* response, void* client_data);

	void handlePointCloud(LivoxLidarEthernetPacket* packet);
	void handleInfoChange(uint32_t handle, const LivoxLidarInfo* info);
	void handleInfoMessage(const std::string& message);
	void publishStatus(const std::string& text);
	void applyPendingDataType(uint32_t handle);

	mutable std::mutex buffer_mutex_;
	std::deque<PointSample> buffer_;
	size_t buffer_limit_;

	mutable std::mutex state_mutex_;
	bool running_;
	bool connected_;
	bool sdk_initialized_;
	std::string config_path_;
	std::string status_text_;
	std::string info_text_;
	std::string serial_number_;
	std::string lidar_ip_;
	uint32_t lidar_handle_;

	std::atomic<uint64_t> total_points_;
	LivoxLidarPointDataType requested_data_type_;
	LivoxLidarPointDataType current_data_type_;
};
