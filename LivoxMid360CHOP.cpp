#include "LivoxMid360CHOP.h"
#include "Parameters.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

namespace
{
	constexpr int kNumOutputChannels = 4;
	constexpr float kRadToDeg = 57.29577951308232f;
}

extern "C"
{

DLLEXPORT
void
FillCHOPPluginInfo(CHOP_PluginInfo* info)
{
	info->apiVersion = CHOPCPlusPlusAPIVersion;
	OP_CustomOPInfo& customInfo = info->customOPInfo;
	customInfo.opType->setString("LivoxMid360");
	customInfo.opLabel->setString("LivoxMid360CHOP");
	customInfo.opIcon->setString("LVX");
	customInfo.authorName->setString("Livox Mid-360 Community");
	customInfo.authorEmail->setString("dev@livox.com");
	customInfo.minInputs = 0;
	customInfo.maxInputs = 0;
}

DLLEXPORT
CHOP_CPlusPlusBase*
CreateCHOPInstance(const OP_NodeInfo* info)
{
	return new LivoxMid360CHOP(info);
}

DLLEXPORT
void
DestroyCHOPInstance(CHOP_CPlusPlusBase* instance)
{
	delete static_cast<LivoxMid360CHOP*>(instance);
}

}

LivoxMid360CHOP::LivoxMid360CHOP(const OP_NodeInfo* info)
	: node_info_(info)
	, execute_count_(0)
	, last_requested_samples_(4096)
	, sample_fill_ratio_(0.0)
	, status_message_("Idle")
	, cached_config_path_()
	, active_config_path_()
	, last_point_mode_(PointDataMenuItems::High)
	, buffer_limit_setting_(200000)
{
}

LivoxMid360CHOP::~LivoxMid360CHOP()
{
	device_.stop();
}

void
LivoxMid360CHOP::getGeneralInfo(CHOP_GeneralInfo* ginfo, const OP_Inputs*, void*)
{
	ginfo->cookEveryFrameIfAsked = true;
	ginfo->timeslice = false;
}

bool
LivoxMid360CHOP::getOutputInfo(CHOP_OutputInfo* info, const OP_Inputs* inputs, void*)
{
	const int samples = std::max(1, Parameters::evalPointsPerFrame(inputs));
	info->numChannels = kNumOutputChannels;
	info->numSamples = samples;
	info->startIndex = 0;
	return true;
}

void
LivoxMid360CHOP::getChannelName(int32_t index, OP_String* name, const OP_Inputs* inputs, void*)
{
	const CoordMenuItems mode = Parameters::evalCoord(inputs);
	if (mode == CoordMenuItems::Cartesian)
	{
		static const std::array<const char*, kNumOutputChannels> labels = { "x", "y", "z", "intensity" };
		name->setString(labels[static_cast<size_t>(index)]);
	}
	else
	{
		static const std::array<const char*, kNumOutputChannels> labels = { "distance", "theta", "phi", "intensity" };
		name->setString(labels[static_cast<size_t>(index)]);
	}
}

int32_t
LivoxMid360CHOP::getNumInfoCHOPChans(void*)
{
	return 3;
}

void
LivoxMid360CHOP::getInfoCHOPChan(int32_t index, OP_InfoCHOPChan* chan, void*)
{
	switch (index)
	{
	case 0:
		chan->name->setString("executions");
		chan->value = static_cast<float>(execute_count_);
		break;
	case 1:
		chan->name->setString("buffered_points");
		chan->value = static_cast<float>(device_.bufferedSamples());
		break;
	case 2:
	default:
		chan->name->setString("fill_ratio");
		chan->value = static_cast<float>(sample_fill_ratio_);
		break;
	}
}

bool
LivoxMid360CHOP::getInfoDATSize(OP_InfoDATSize* infoSize, void*)
{
	infoSize->cols = 2;
	infoSize->rows = 7;
	infoSize->byColumn = false;
	return true;
}

void
LivoxMid360CHOP::getInfoDATEntries(int32_t index, int32_t, OP_InfoDATEntries* entries, void*)
{
	const auto setEntry = [&](const char* label, const std::string& value)
	{
		entries->values[0]->setString(label);
		entries->values[1]->setString(value.c_str());
	};

	switch (index)
	{
	case 0:
		setEntry("Status", status_message_);
		break;
	case 1:
		setEntry("Config Path", cached_config_path_);
		break;
	case 2:
		setEntry("Serial", device_.lidarSerial());
		break;
	case 3:
		setEntry("Lidar IP", device_.lidarIp());
		break;
	case 4:
		setEntry("Buffered samples", std::to_string(device_.bufferedSamples()));
		break;
	case 5:
		setEntry("Total samples", std::to_string(device_.totalPoints()));
		break;
	case 6:
	default:
		setEntry("Info message", device_.infoMessage());
		break;
	}
}

void
LivoxMid360CHOP::execute(CHOP_Output* output, const OP_Inputs* inputs, void*)
{
	execute_count_++;
	last_requested_samples_ = static_cast<size_t>(std::max(1, Parameters::evalPointsPerFrame(inputs)));

	const size_t desired_buffer = static_cast<size_t>(std::max(Parameters::evalBufferLimit(inputs), static_cast<int>(last_requested_samples_)));
	if (desired_buffer != buffer_limit_setting_)
	{
		buffer_limit_setting_ = desired_buffer;
		device_.setBufferLimit(buffer_limit_setting_);
	}

	ensureState(inputs);
	updateDataType(Parameters::evalPointData(inputs));

	const CoordMenuItems coord = Parameters::evalCoord(inputs);
	fillChannels(output, coord, last_requested_samples_);

	status_message_ = device_.statusText();
}

void
LivoxMid360CHOP::setupParameters(OP_ParameterManager* manager, void*)
{
	Parameters::setup(manager);
}

void
LivoxMid360CHOP::pulsePressed(const char* name, void*)
{
	if (strcmp(name, ResetName) == 0)
	{
		device_.clear();
	}
}

void
LivoxMid360CHOP::ensureState(const OP_Inputs* inputs)
{
	const bool should_run = Parameters::evalActive(inputs) != 0;
	const std::string config_path = inputs->getParString(ConfigPathName);
	cached_config_path_ = config_path;

	if (!should_run && device_.isRunning())
	{
		device_.stop();
		active_config_path_.clear();
	}
	else if (should_run)
	{
		const bool needs_restart = (!device_.isRunning()) || (config_path != active_config_path_);
		if (needs_restart)
		{
			if (device_.isRunning())
			{
				device_.stop();
			}
			if (device_.start(config_path))
			{
				active_config_path_ = config_path;
			}
		}
	}

}

void
LivoxMid360CHOP::updateDataType(PointDataMenuItems data_mode)
{
	if (data_mode == last_point_mode_)
	{
		return;
	}

	last_point_mode_ = data_mode;
	if (data_mode == PointDataMenuItems::High)
	{
		device_.setPointDataType(kLivoxLidarCartesianCoordinateHighData);
	}
	else
	{
		device_.setPointDataType(kLivoxLidarCartesianCoordinateLowData);
	}
}

size_t
LivoxMid360CHOP::fillChannels(CHOP_Output* output, CoordMenuItems coord_mode, size_t requested_samples)
{
	const size_t safe_samples = std::min(requested_samples, static_cast<size_t>(output->numSamples));
	std::vector<LivoxDevice::PointSample> samples(safe_samples);
	const size_t populated = device_.consume(samples.data(), safe_samples);

	for (size_t s = 0; s < safe_samples; ++s)
	{
		float c0 = 0.0f;
		float c1 = 0.0f;
		float c2 = 0.0f;
		float c3 = 0.0f;

		if (s < populated)
		{
			const auto& sample = samples[s];
			if (coord_mode == CoordMenuItems::Cartesian)
			{
				c0 = sample.x;
				c1 = sample.y;
				c2 = sample.z;
				c3 = sample.intensity;
			}
			else
			{
				const float horizontal = std::sqrt(sample.x * sample.x + sample.y * sample.y);
				const float distance = std::sqrt(horizontal * horizontal + sample.z * sample.z);
				const float theta = std::atan2(sample.y, sample.x) * kRadToDeg;
				const float phi = std::atan2(sample.z, horizontal) * kRadToDeg;
				c0 = distance;
				c1 = theta;
				c2 = phi;
				c3 = sample.intensity;
			}
		}

		for (int ch = 0; ch < kNumOutputChannels; ++ch)
		{
			float value = 0.0f;
			if (ch == 0) value = c0;
			else if (ch == 1) value = c1;
			else if (ch == 2) value = c2;
			else value = c3;

			output->channels[ch][s] = value;
		}
	}

	sample_fill_ratio_ = safe_samples == 0 ? 0.0 : static_cast<double>(populated) / static_cast<double>(safe_samples);
	return populated;
}
