#include "PCH.h"
#include "SensorNode.h"

namespace FT_TDMA_Sim
{
#if 0
	SensorNode::SensorNode(const SensorNode& other)
	{
		m_ID = other.;
		m_Position = other.;
		m_Parent = other.;
		m_Level = other.;

		m_DeltaOpt = other.;

		m_CurrentData = other.;

		m_CollectionTime = other.;
		m_WastedTime = other.;

		m_EnergyConsumed = other.;

		m_SentPacketTotalDelay = other.;
		m_SentPacketCount = other.;

		m_Color = other.;

		m_TotalDataSent = other.;

		m_Packets = other.;

		m_CurrentPacketIterator = other.;

		m_CurrentParent = other.;
		m_CurrentColor = other.;

		m_WelshPowellDegree = other.;

		m_FailureDistribution = other.;
		m_FailureTimestamps = other.;
		m_FailureIterator = other.;
	}
#endif

	std::string WorkingStateToString(const WorkingState& ws)
	{
		switch (ws)
		{
		case WorkingState::Transfer: 
			return "Transfer";
		case WorkingState::Collection: 
			return "Collection";
		case WorkingState::Recovery: 
			return "Recovery";
		}

		throw std::runtime_error("Unknown Working State in WorkingStateToString!");
		return "";
	}

	void SensorNode::Reset()
	{
		m_Parent = c_InvalidIndex;
		m_Level = -1;

		m_DeltaOpt = 0;

		m_CurrentData = 0;

		m_CollectionTime = 0;
		m_WastedTime = 0;

		m_EnergyConsumed = 0;

		m_SentPacketTotalDelay = 0;
		m_SentPacketCount = 0;

		m_Color = -1;

		m_TotalDataSent = 0;

		m_Packets.clear();

		m_CurrentPacketIterator = -1;

		m_CurrentParent = -1;
		m_CurrentColor = -1;

		m_WelshPowellDegree = -1;

		m_FailureIterator = 0;
	}
}