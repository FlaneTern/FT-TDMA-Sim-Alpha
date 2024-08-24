#include "PCH.h"
#include "Simulator.h"


namespace FT_TDMA_Sim
{

	//static constexpr double s_EnergyRateWorking = 0.4;
	//static constexpr double s_EnergyRateDataTransfer = 1.0;
	//static constexpr double s_EnergyTransitionWorkingToTransfer = 20.0;
	//static constexpr double s_EnergyTransitionTransferToWorking = 20.0;

	static constexpr double s_EnergyTransitionWorkingToTransfer = 0.0;
	static constexpr double s_EnergyTransitionTransferToWorking = 0.0;

	//std::vector<SimulationSummaryData> Simulator::s_Summary;

	//template<typename T>
	//std::vector<std::shared_ptr<Simulator>> Simulator::CreateSimulator(SimulatorParameterGrid spg)
	//{
	//	std::vector<std::shared_ptr<Simulator>> simulators;

	//	for (auto& recoveryTime : spg.RecoveryTime)
	//	{
	//		for (auto& transferTime : spg.TransferTime)
	//		{
	//			for (auto& totalDurationToBeTransferred : spg.TotalDurationToBeTransferred)
	//			{
	//				for (auto& energyRateWorking : spg.EnergyRateWorking)
	//				{
	//					for (int energyRateTransfer : spg.EnergyRateTransfer)
	//					{
	//						for (auto& transmissionRange : spg.TransmissionRange)
	//						{
	//							for (auto& interferenceRange : spg.InterferenceRange)
	//							{
	//								FT_TDMA_Sim::SimulatorParameters sp =
	//								{
	//									totalDurationToBeTransferred,
	//									transferTime,
	//									recoveryTime,
	//									energyRateWorking,
	//									energyRateTransfer,
	//									transmissionRange,
	//									interferenceRange
	//								};

	//								simulators.push_back(std::make_shared<T>(sp));
	//							}
	//						}
	//					}
	//				}
	//			}
	//		}
	//	}

	//	return simulators;
	//}

	Simulator::Simulator(SimulatorParameters sp)
		: m_SimulatorParameters(sp)
	{
		static int64_t currentSimulationID = 1;

		currentSimulationID++;
		m_SimulationID = currentSimulationID;
	}

	void Simulator::Run(const std::vector<SensorNode>& SNs)
	{
		m_SensorNodes = SNs;

		ConstructTopology();
		ConstructTopologyPost();
		ColorTopology();
		ColorTopologyPost();
		SetSNDeltas();
		SetSNDeltasPost();

		Simulate();
	}

	void Simulator::ConstructTopology()
	{

		// find parent

		for (int i = 0; i < m_SensorNodes.size(); i++)
		{
			if (std::sqrt(m_SensorNodes[i].m_Position.X * m_SensorNodes[i].m_Position.X +
				m_SensorNodes[i].m_Position.Y * m_SensorNodes[i].m_Position.Y) < m_SimulatorParameters.TransmissionRange)
			{
				m_SensorNodes[i].m_Parent = SensorNode::c_BaseStationIndex;
				m_SensorNodes[i].m_Level = 0;
			}
		}
		
		int currentLevel = 1;
		bool done = false;
		bool assigned = true;


		while (!done && assigned)
		{
			done = true;

			for (int i = 0; i < m_SensorNodes.size(); i++)
			{
				if (m_SensorNodes[i].m_Parent != SensorNode::c_InvalidIndex)
					continue;

				assigned = false;


				struct IDDistance
				{
					int64_t SNID;
					double Distance;
				};

				std::vector<IDDistance> temp;

				for (int j = 0; j < m_SensorNodes.size(); j++)
				{
					if (m_SensorNodes[i].m_Level != currentLevel - 1)
						continue;

					double distance = SensorNode::Distance(m_SensorNodes[i], m_SensorNodes[j]);

					if (distance < m_SimulatorParameters.TransmissionRange)
						temp.push_back({ (int64_t)j, distance });

				}

				if (temp.empty())
				{
					done = false;
					continue;
				}
				assigned = true;


				int closestIndex = -1;
				double closestDistance = std::numeric_limits<double>::max();
				for (int j = 0; j < temp.size(); j++)
				{
					if (closestDistance > temp[j].Distance)
					{
						closestIndex = temp[j].SNID;
						closestDistance = temp[j].Distance;
					}
				}

				m_SensorNodes[i].m_Parent = closestIndex;
			}


			currentLevel++;
		}


		for (int i = 0; i < m_SensorNodes.size(); i++)
		{
			if (m_SensorNodes[i].m_Parent == SensorNode::c_InvalidIndex)
				m_SensorNodes[i].m_Parent == SensorNode::c_NoParentIndex;
		}

	}

	void Simulator::ColorTopology()
	{
		for (int i = 0; i < m_SensorNodes.size(); i++)
		{
			for (int j = 0; j < m_SensorNodes.size(); j++)
			{
				if (SensorNode::Distance(m_SensorNodes[i], m_SensorNodes[j]) <= m_SimulatorParameters.InterferenceRange)
				{
					if (m_SensorNodes[i].m_WelshPowellDegree == -1)
						m_SensorNodes[i].m_WelshPowellDegree = 0;
					m_SensorNodes[i].m_WelshPowellDegree++;
				}
			}
		}

		std::sort(m_SensorNodes.begin(), m_SensorNodes.end(), [&](SensorNode sn1, SensorNode sn2) {
			return sn1.m_WelshPowellDegree > sn2.m_WelshPowellDegree;
			});

		bool exists = true;
		for (int i = 0; exists; i++)
		{
			exists = false;
			for (int j = 0; j < m_SensorNodes.size(); j++)
			{
				if (m_SensorNodes[j].m_Color != -1)
					continue;
				bool con = false;
				for (int k = 0; k < m_SensorNodes.size(); k++)
					if (m_SensorNodes[k].m_Color == i && SensorNode::Distance(m_SensorNodes[j], m_SensorNodes[k]) <= m_SimulatorParameters.InterferenceRange) {
						con = true;
						break;
					}
				if (!con) {
					m_SensorNodes[j].m_Color = i;
					exists = true;
				}
			}
		}

		std::sort(m_SensorNodes.begin(), m_SensorNodes.end(), [&](SensorNode sn1, SensorNode sn2) {
			return sn1.m_ID > sn2.m_ID;
			});
	}

	void Simulator::SetSNDeltas()
	{
		Distribution uniformDist(DistributionType::Uniform, 1000.0, 1.0);
		
		for (int i = 0; i < m_SensorNodes.size(); i++)
			m_SensorNodes[i].m_DeltaOpt = uniformDist.GenerateRandomNumber();
	}

	void Simulator::ConstructTopologyPost()
	{
		for (int i = 0; i < m_SensorNodes.size(); i++)
			m_SensorNodes[i].m_CurrentParent = m_SensorNodes[i].m_Parent;

		// TO DO : CHECK VALIDITY OF TOPOLOGY
	}

	void Simulator::ColorTopologyPost()
	{
		for (int i = 0; i < m_SensorNodes.size(); i++)
			m_SensorNodes[i].m_CurrentColor = m_SensorNodes[i].m_Color;

		// TO DO : CHECK VALIDITY OF TOPOLOGY
	}

	void Simulator::SetSNDeltasPost()
	{

	}

	void Simulator::Simulate()
	{
		SimulationResults& sr = m_SimulationResults;
			
		struct WorkingStateTimestamp
		{
			int64_t SNID;
			WorkingState State;
			double Timestamp;
		};

		std::vector<WorkingStateTimestamp> previousEvents;
		auto pqCompare = [](WorkingStateTimestamp left, WorkingStateTimestamp right) 
		{ 
			if (left.Timestamp > right.Timestamp)
				return true;
			else if (left.Timestamp < right.Timestamp)
				return false;

			return left.SNID < right.SNID;
		};

		std::priority_queue<WorkingStateTimestamp, std::vector<WorkingStateTimestamp>, decltype(pqCompare)> eventQueue(pqCompare);
		for (int i = 0; i < m_SensorNodes.size(); i++)
		{
			eventQueue.push({ (int64_t)i, WorkingState::Collection, 0.0 });
			previousEvents.push_back({ (int64_t)i, WorkingState::Collection, 0.0 });
		}
		

		int colorCount = -1;
		for (int i = 0; i < m_SensorNodes.size(); i++)
			colorCount = std::max(colorCount, (int)m_SensorNodes[i].m_Color);
		colorCount++;
		std::cout << "colorCount = " << colorCount << '\n';

		double transferredTotalDuration = 0;
		double currentTime = 0.0;
		int failureCount = 0;

		int superSlotIterator = 0;

		bool isDone = false;

		while (!isDone)
		{
			auto currentEvent = eventQueue.top();
			currentTime = currentEvent.Timestamp;
			auto& currentState = currentEvent.State;
			auto& currentSN = currentEvent.SNID;
			eventQueue.pop();

			//std::cout << "Current Time = " << currentTime << '\n';

			superSlotIterator = currentTime / (m_SimulatorParameters.TransferTime * colorCount);

			//std::cout << "here = " << currentSN << '\n';
			//if(eventQueue.size() > 99)
			//	std::cout << "eventQueue.size() " << eventQueue.size() << '\n';


			// deciding the next state to put in eventQueue
			{
				double nextTime = currentTime;
				WorkingState nextState;
				if (currentState == WorkingState::Collection)
				{
						//double optimalTime = currentTime + m_SensorNodes[currentSN].m_DeltaOpt;
						//double clammedOptimalTime = (int)((currentTime) / (m_SensorNodes[currentSN].m_DeltaOpt + m_SimulatorParameters.TransferTime)) * (m_SensorNodes[currentSN].m_DeltaOpt + m_SimulatorParameters.TransferTime) + m_SensorNodes[currentSN].m_DeltaOpt;
						

#if 0
						// now newtime is equal to timeslotstart
						nextTime = (int)((currentTime + m_SensorNodes[currentSN].m_DeltaOpt) / (m_SimulatorParameters.TransferTime * colorCount)) * m_SimulatorParameters.TransferTime * colorCount + m_SensorNodes[currentSN].m_CurrentColor * m_SimulatorParameters.TransferTime;

						while(nextTime < currentTime)
							nextTime += m_SimulatorParameters.TransferTime * colorCount;

						//while (std::abs(nextTime - (currentTime + m_SensorNodes[currentSN].m_DeltaOpt)) > 0.5 * m_SimulatorParameters.TransferTime * colorCount &&
						//	std::abs(nextTime + m_SimulatorParameters.TransferTime * colorCount - (currentTime + m_SensorNodes[currentSN].m_DeltaOpt)) < std::abs(nextTime - (currentTime + m_SensorNodes[currentSN].m_DeltaOpt)))
						while (std::abs(nextTime + m_SimulatorParameters.TransferTime * colorCount - (currentTime + m_SensorNodes[currentSN].m_DeltaOpt)) < std::abs(nextTime - (currentTime + m_SensorNodes[currentSN].m_DeltaOpt)))
							nextTime += m_SimulatorParameters.TransferTime * colorCount;
#elif 0
						
						
						//// int of the optimaliterator
						//double optimalTime = (int)((currentTime) / (m_SensorNodes[currentSN].m_DeltaOpt + m_SimulatorParameters.TransferTime)) * (m_SensorNodes[currentSN].m_DeltaOpt + m_SimulatorParameters.TransferTime) + m_SensorNodes[currentSN].m_DeltaOpt;
						//// int of the super time slot
						//nextTime = (int)((optimalTime) / (m_SimulatorParameters.TransferTime * colorCount)) * m_SimulatorParameters.TransferTime * colorCount + m_SensorNodes[currentSN].m_CurrentColor * m_SimulatorParameters.TransferTime;
						//while(nextTime < currentTime)
						//	nextTime += m_SimulatorParameters.TransferTime * colorCount;
						//while(std::abs(nextTime - optimalTime) > std::abs(nextTime + m_SimulatorParameters.TransferTime * colorCount - optimalTime))
						//	nextTime += m_SimulatorParameters.TransferTime * colorCount;
#elif 0
						double optimalTime = currentTime + m_SensorNodes[currentSN].m_DeltaOpt;
						nextTime = m_SensorNodes[currentSN].m_CurrentColor * m_SimulatorParameters.TransferTime;
						if (nextTime < currentTime)
							nextTime += (int)((currentTime - nextTime) / (m_SimulatorParameters.TransferTime * colorCount)) * m_SimulatorParameters.TransferTime * colorCount;
						while (nextTime >= currentTime)
							nextTime -= m_SimulatorParameters.TransferTime * colorCount;
						nextTime += m_SimulatorParameters.TransferTime * colorCount;
						while(std::abs(nextTime - optimalTime) > std::abs(nextTime + m_SimulatorParameters.TransferTime * colorCount - optimalTime))
							nextTime += m_SimulatorParameters.TransferTime * colorCount;
#elif 1
						double optimalTime = currentTime + m_SensorNodes[currentSN].m_DeltaOpt;
						nextTime = m_SensorNodes[currentSN].m_CurrentColor * m_SimulatorParameters.TransferTime;
						if (nextTime < currentTime)
							nextTime += (int)((currentTime - nextTime) / (m_SimulatorParameters.TransferTime * colorCount)) * m_SimulatorParameters.TransferTime * colorCount;
						while (nextTime >= currentTime)
							nextTime -= m_SimulatorParameters.TransferTime * colorCount;
						nextTime += m_SimulatorParameters.TransferTime * colorCount;
						while (nextTime < optimalTime)
							nextTime += m_SimulatorParameters.TransferTime * colorCount;
#endif

					nextState = WorkingState::Transfer;

					//std::cout << "a = " << nextTime << '\n';
					//std::cout << "b = " << m_SensorNodes[currentSN].m_CurrentColor * m_SimulatorParameters.TransferTime << '\n';
					//std::cout << "c = " << (int)((currentTime + m_SensorNodes[currentSN].m_DeltaOpt) / (m_SimulatorParameters.TransferTime * colorCount)) << '\n';
					//std::cout << "d = " << m_SensorNodes[currentSN].m_CurrentColor << '\n';
					//std::cout << "e = " << colorCount << '\n';
					//std::cout << "Next Slot = " << nextTime << '\n';
				}
				else if (currentState == WorkingState::Transfer)
				{
					nextTime += m_SimulatorParameters.TransferTime;
					nextState = WorkingState::Collection;
				}
				else if (currentState == WorkingState::Recovery)
				{
					nextTime += m_SimulatorParameters.RecoveryTime;
					nextState = WorkingState::Collection;
				}

				if (m_SensorNodes[currentSN].m_FailureIterator < m_SensorNodes[currentSN].m_FailureTimestamps.size() &&
					nextTime >= m_SensorNodes[currentSN].m_FailureTimestamps[m_SensorNodes[currentSN].m_FailureIterator])
				{
					eventQueue.push({ currentSN, WorkingState::Recovery, m_SensorNodes[currentSN].m_FailureTimestamps[m_SensorNodes[currentSN].m_FailureIterator] });
					m_SensorNodes[currentSN].m_FailureIterator++;
				}
				else
				{
					eventQueue.push({ currentSN, nextState, nextTime });
				}
			}

			// still no rerouting
			if (previousEvents[currentSN].State == WorkingState::Collection)
			{
				if (currentState == WorkingState::Collection) // handles the initialization
				{
					m_SensorNodes[currentSN].m_Packets.push_back({ currentSN, currentTime });
					m_SensorNodes[currentSN].m_CurrentPacketIterator = m_SensorNodes[currentSN].m_Packets.size() - 1;
				}
				else if (currentState == WorkingState::Transfer)
				{
					m_SensorNodes[currentSN].m_CollectionTime += currentTime - previousEvents[currentSN].Timestamp;
					m_SensorNodes[currentSN].m_CurrentData += currentTime - previousEvents[currentSN].Timestamp;
					m_SensorNodes[currentSN].m_EnergyConsumed += (currentTime - previousEvents[currentSN].Timestamp) * m_SimulatorParameters.EnergyRateWorking + s_EnergyTransitionWorkingToTransfer;
					//m_SensorNodes[currentSN].m_Packets[m_SensorNodes[currentSN].m_CurrentPacketIterator].Size += currentTime - previousEvents[currentSN].Timestamp; // look at this
					m_SensorNodes[currentSN].m_Packets[m_SensorNodes[currentSN].m_CurrentPacketIterator].Size += currentTime - m_SensorNodes[currentSN].m_Packets[m_SensorNodes[currentSN].m_CurrentPacketIterator].InitialTimestamp; // look at this
				}
				else if (currentState == WorkingState::Recovery)
				{
					m_SensorNodes[currentSN].m_WastedTime += currentTime - previousEvents[currentSN].Timestamp;
					failureCount++;
					m_SensorNodes[currentSN].m_EnergyConsumed += (currentTime - previousEvents[currentSN].Timestamp) * m_SimulatorParameters.EnergyRateWorking;
					m_SensorNodes[currentSN].m_Packets.clear();
					m_SensorNodes[currentSN].m_CurrentPacketIterator = - 1;
				}
			}
			else if (previousEvents[currentSN].State == WorkingState::Transfer)
			{
				if (currentState == WorkingState::Collection)
				{
					if(m_SensorNodes[currentSN].m_CurrentParent != SensorNode::c_BaseStationIndex)
					{
						if (previousEvents[m_SensorNodes[currentSN].m_CurrentParent].State != WorkingState::Recovery)
						{
							m_SensorNodes[m_SensorNodes[currentSN].m_CurrentParent].m_CurrentData += m_SensorNodes[currentSN].m_CurrentData;
							for (int i = 0; i < m_SensorNodes[currentSN].m_Packets.size(); i++)
								m_SensorNodes[m_SensorNodes[currentSN].m_CurrentParent].m_Packets.push_back(m_SensorNodes[currentSN].m_Packets[i]);
						}
					}
					else
					{
						transferredTotalDuration += m_SensorNodes[currentSN].m_CurrentData;
						for (int i = 0; i < m_SensorNodes[currentSN].m_Packets.size(); i++)
						{
							m_SensorNodes[m_SensorNodes[currentSN].m_Packets[i].InitialSNID].m_SentPacketTotalDelay += currentTime - m_SensorNodes[currentSN].m_Packets[i].InitialTimestamp;
							m_SensorNodes[m_SensorNodes[currentSN].m_Packets[i].InitialSNID].m_SentPacketCount++;

							m_SensorNodes[m_SensorNodes[currentSN].m_Packets[i].InitialSNID].m_TotalDataSent += m_SensorNodes[currentSN].m_Packets[i].Size;
						}

					}

					if (m_SensorNodes[currentSN].m_CurrentParent == SensorNode::c_BaseStationIndex || previousEvents[m_SensorNodes[currentSN].m_CurrentParent].State != WorkingState::Recovery)
					{
						m_SensorNodes[currentSN].m_Packets.clear();
						m_SensorNodes[currentSN].m_CurrentData = 0;
					}

					m_SensorNodes[currentSN].m_WastedTime += m_SimulatorParameters.TransferTime;
					m_SensorNodes[currentSN].m_EnergyConsumed += m_SimulatorParameters.TransferTime * m_SimulatorParameters.EnergyRateTransfer + s_EnergyTransitionTransferToWorking;
					m_SensorNodes[currentSN].m_Packets.push_back({ currentSN, currentTime });
					m_SensorNodes[currentSN].m_CurrentPacketIterator = m_SensorNodes[currentSN].m_Packets.size() - 1;
				}
				else if (currentState == WorkingState::Transfer) {} // not possible
				else if (currentState == WorkingState::Recovery) // WARNING : PARTIAL TRANSFER FAILS	
				{
					m_SensorNodes[currentSN].m_CurrentData = 0;
					m_SensorNodes[currentSN].m_WastedTime += currentTime - previousEvents[currentSN].Timestamp;
					failureCount++;
					m_SensorNodes[currentSN].m_EnergyConsumed += (currentTime - previousEvents[currentSN].Timestamp) * m_SimulatorParameters.EnergyRateTransfer;
					m_SensorNodes[currentSN].m_Packets.clear();
					m_SensorNodes[currentSN].m_CurrentPacketIterator = - 1;

				}
			}
			else if (previousEvents[currentSN].State == WorkingState::Recovery)
			{

				if (currentState == WorkingState::Collection)
				{
					m_SensorNodes[currentSN].m_WastedTime += m_SimulatorParameters.RecoveryTime;
					m_SensorNodes[currentSN].m_Packets.push_back({ currentSN, currentTime });
					m_SensorNodes[currentSN].m_CurrentPacketIterator = m_SensorNodes[currentSN].m_Packets.size() - 1;
					
				}
				else if (currentState == WorkingState::Transfer) {} // not possible
				else if (currentState == WorkingState::Recovery)
				{
					m_SensorNodes[currentSN].m_WastedTime += currentTime - previousEvents[currentSN].Timestamp;
					failureCount++;
					m_SensorNodes[currentSN].m_Packets.clear();
					m_SensorNodes[currentSN].m_CurrentPacketIterator =  - 1;
				}
			}

			// throw std::runtime_error("Exceeded the last failure point!");;
			previousEvents[currentSN] = currentEvent;
			
			//if (m_SensorNodes[currentSN].m_Packets.size() > 1000)
			//	std::cout << "m_SensorNodes[currentSN].m_Packets.size() = " << m_SensorNodes[currentSN].m_Packets.size() << '\n';


			isDone = IsDone(currentTime);
		}

		sr.ActualTotalDuration = currentTime;
		sr.FinalFailureIndex = failureCount;

		for (int i = 0; i < m_SensorNodes.size(); i++)
		{
			std::cout << "SN " << i << "\tDelta = " << m_SensorNodes[i].m_DeltaOpt << '\t';
			std::cout << "Collection Time = " << m_SensorNodes[i].m_CollectionTime << '\t';
			std::cout << "Wasted Time = " << m_SensorNodes[i].m_WastedTime << '\t';
			std::cout << "EnergyConsumed = " << m_SensorNodes[i].m_EnergyConsumed << '\n';
		}
		std::cout << "Actual Total Duration = " << sr.ActualTotalDuration << '\n';

	}

	bool Simulator::IsDone(double currentTime)
	{
		return currentTime >= m_SimulatorParameters.TotalDurationToBeTransferred;
	}

	void Simulator::Reroute()
	{

	}

}