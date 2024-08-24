#pragma once
#include "Distribution.h"
#include "DataInterface.h"

namespace FT_TDMA_Sim
{
	struct Position
	{
		double X = 0;
		double Y = 0;
	};

	// Sensor Node states
	enum class WorkingState
	{
		Collection,
		Transfer,
		Recovery
	};

	std::string WorkingStateToString(const WorkingState& ws);

	struct Packet
	{
		int64_t InitialSNID;
		double InitialTimestamp;
		double Size;
	};

	class SensorNode
	{
	public:
		//SensorNode(const SensorNode& other);

		int64_t m_ID = -1;
		Position m_Position;
		int64_t m_Parent = c_InvalidIndex;
		int64_t m_Level = -1;

		double m_DeltaOpt = 0;

		double m_CurrentData = 0;

		double m_CollectionTime = 0;
		double m_WastedTime = 0;

		double m_EnergyConsumed = 0;

		double m_SentPacketTotalDelay = 0;
		int64_t m_SentPacketCount = 0;

		int64_t m_Color = -1;

		double m_TotalDataSent = 0;

		std::vector<Packet> m_Packets;

		int m_CurrentPacketIterator = -1;

		int64_t m_CurrentParent = -1;
		int64_t m_CurrentColor = -1;

		int64_t m_WelshPowellDegree = -1;

		std::vector<double> m_FailureTimestamps;
		int64_t m_FailureIterator = 0;

		I_SensorNodeData i_SensorNodeData;

		void Reset();


		static const int64_t c_BaseStationIndex = -1;
		static const int64_t c_NoParentIndex = -2;
		static const int64_t c_InvalidIndex = -3;


		static inline double Distance(SensorNode a, SensorNode b) 
		{
			return std::sqrt(
				(a.m_Position.X - b.m_Position.X) * (a.m_Position.X - b.m_Position.X) +
				(a.m_Position.Y - b.m_Position.Y) * (a.m_Position.Y - b.m_Position.Y)
			);
		}
	};
}