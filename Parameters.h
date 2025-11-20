#pragma once

class OP_Inputs;
class OP_ParameterManager;

// Parameter names and labels
constexpr static char PageConnectionName[] = "Connection";
constexpr static char PageStreamingName[] = "Streaming";
constexpr static char PageOutputName[] = "Output";

constexpr static char ActiveName[] = "Active";
constexpr static char ActiveLabel[] = "Active";

constexpr static char ConfigPathName[] = "Configpath";
constexpr static char ConfigPathLabel[] = "Config File";

constexpr static char PointsPerFrameName[] = "Pointsperframe";
constexpr static char PointsPerFrameLabel[] = "Points Per Cook";

constexpr static char BufferLimitName[] = "Bufferlimit";
constexpr static char BufferLimitLabel[] = "Buffer Limit";

constexpr static char DataTypeName[] = "Datatype";
constexpr static char DataTypeLabel[] = "Point Data Type";

constexpr static char CoordName[] = "Coordmode";
constexpr static char CoordLabel[] = "Coordinate Output";

constexpr static char ResetName[] = "Resetbuffer";
constexpr static char ResetLabel[] = "Reset Buffer";

enum class CoordMenuItems
{
	Cartesian = 0,
	Spherical = 1
};

enum class PointDataMenuItems
{
	High = 0,
	Low = 1
};

class Parameters
{
public:
	static void setup(OP_ParameterManager* manager);

	static int evalActive(const OP_Inputs* input);
	static int evalPointsPerFrame(const OP_Inputs* input);
	static int evalBufferLimit(const OP_Inputs* input);
	static CoordMenuItems evalCoord(const OP_Inputs* input);
	static PointDataMenuItems evalPointData(const OP_Inputs* input);
};
