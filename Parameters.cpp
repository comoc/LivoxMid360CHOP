#include <array>
#include <cassert>
#include "CPlusPlus_Common.h"
#include "Parameters.h"

int
Parameters::evalActive(const OP_Inputs* input)
{
	return input->getParInt(ActiveName);
}

int
Parameters::evalPointsPerFrame(const OP_Inputs* input)
{
	return input->getParInt(PointsPerFrameName);
}

int
Parameters::evalBufferLimit(const OP_Inputs* input)
{
	return input->getParInt(BufferLimitName);
}

CoordMenuItems
Parameters::evalCoord(const OP_Inputs* input)
{
	return static_cast<CoordMenuItems>(input->getParInt(CoordName));
}

PointDataMenuItems
Parameters::evalPointData(const OP_Inputs* input)
{
	return static_cast<PointDataMenuItems>(input->getParInt(DataTypeName));
}

void
Parameters::setup(OP_ParameterManager* manager)
{
	// Active toggle
	{
		OP_NumericParameter np;
		np.name = ActiveName;
		np.label = ActiveLabel;
		np.page = PageConnectionName;
		np.defaultValues[0] = 0;
		const OP_ParAppendResult res = manager->appendToggle(np);
		assert(res == OP_ParAppendResult::Success);
	}

	// Config path
	{
		OP_StringParameter sp;
		sp.name = ConfigPathName;
		sp.label = ConfigPathLabel;
		sp.page = PageConnectionName;
		sp.defaultValue = "D:/Livox/Livox-SDK2/samples/livox_lidar_quick_start/mid360_config.json";
		const OP_ParAppendResult res = manager->appendFile(sp);
		assert(res == OP_ParAppendResult::Success);
	}

	// Points per frame
	{
		OP_NumericParameter np;
		np.name = PointsPerFrameName;
		np.label = PointsPerFrameLabel;
		np.page = PageStreamingName;
		np.defaultValues[0] = 4096;
		np.minValues[0] = 64;
		np.clampMins[0] = true;
		np.maxValues[0] = 65536;
		np.clampMaxes[0] = true;
		const OP_ParAppendResult res = manager->appendInt(np);
		assert(res == OP_ParAppendResult::Success);
	}

	// Buffer limit
	{
		OP_NumericParameter np;
		np.name = BufferLimitName;
		np.label = BufferLimitLabel;
		np.page = PageStreamingName;
		np.defaultValues[0] = 200000;
		np.minValues[0] = 1024;
		np.clampMins[0] = true;
		const OP_ParAppendResult res = manager->appendInt(np);
		assert(res == OP_ParAppendResult::Success);
	}

	// Data type menu
	{
		OP_StringParameter sp;
		sp.name = DataTypeName;
		sp.label = DataTypeLabel;
		sp.page = PageOutputName;
		sp.defaultValue = "High";
		std::array<const char*, 2> names = { "High", "Low" };
		std::array<const char*, 2> labels = { "Cartesian High (mm)", "Cartesian Low (cm)" };
		const OP_ParAppendResult res = manager->appendMenu(sp, static_cast<int>(names.size()), names.data(), labels.data());
		assert(res == OP_ParAppendResult::Success);
	}

	// Coordinate system
	{
		OP_StringParameter sp;
		sp.name = CoordName;
		sp.label = CoordLabel;
		sp.page = PageOutputName;
		sp.defaultValue = "Cartesian";
		std::array<const char*, 2> names = { "Cartesian", "Spherical" };
		std::array<const char*, 2> labels = { "Cartesian", "Spherical" };
		const OP_ParAppendResult res = manager->appendMenu(sp, static_cast<int>(names.size()), names.data(), labels.data());
		assert(res == OP_ParAppendResult::Success);
	}

	// Reset pulse
	{
		OP_NumericParameter np;
		np.name = ResetName;
		np.label = ResetLabel;
		np.page = PageStreamingName;
		const OP_ParAppendResult res = manager->appendPulse(np);
		assert(res == OP_ParAppendResult::Success);
	}
}
