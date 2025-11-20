#pragma once

#include <string>

#include "CHOP_CPlusPlusBase.h"
#include "Parameters.h"
#include "LivoxDevice.h"

class LivoxMid360CHOP : public CHOP_CPlusPlusBase
{
public:
	explicit LivoxMid360CHOP(const OP_NodeInfo* info);
	~LivoxMid360CHOP() override;

	void getGeneralInfo(CHOP_GeneralInfo* ginfo, const OP_Inputs* inputs, void* reserved) override;
	bool getOutputInfo(CHOP_OutputInfo* info, const OP_Inputs* inputs, void* reserved) override;
	void getChannelName(int32_t index, OP_String* name, const OP_Inputs* inputs, void* reserved) override;

	int32_t getNumInfoCHOPChans(void* reserved) override;
	void getInfoCHOPChan(int32_t index, OP_InfoCHOPChan* chan, void* reserved) override;

	bool getInfoDATSize(OP_InfoDATSize* infoSize, void* reserved) override;
	void getInfoDATEntries(int32_t index, int32_t nEntries, OP_InfoDATEntries* entries, void* reserved) override;

	void execute(CHOP_Output* output, const OP_Inputs* inputs, void* reserved) override;
	void setupParameters(OP_ParameterManager* manager, void* reserved) override;
	void pulsePressed(const char* name, void* reserved) override;

private:
	void ensureState(const OP_Inputs* inputs);
	void updateDataType(PointDataMenuItems data_mode);
	size_t fillChannels(CHOP_Output* output, CoordMenuItems coord_mode, size_t requested_samples);

	const OP_NodeInfo* node_info_;
	LivoxDevice device_;
	int32_t execute_count_;
	size_t last_requested_samples_;
	double sample_fill_ratio_;
	std::string status_message_;
	std::string cached_config_path_;
	std::string active_config_path_;
	PointDataMenuItems last_point_mode_;
	size_t buffer_limit_setting_;
};
