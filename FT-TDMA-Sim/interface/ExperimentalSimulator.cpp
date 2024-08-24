#include "PCH.h"
#include "ExperimentalSimulator.h"


namespace FT_TDMA_Sim
{

	static constexpr double s_EnergyTransitionWorkingToTransfer = 0.0;
	static constexpr double s_EnergyTransitionTransferToWorking = 0.0;

	ExperimentalSimulator::ExperimentalSimulator(SimulatorParameters sp)
		: Simulator(sp)
	{

	}

	void ExperimentalSimulator::ConstructTopology()
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

	void ExperimentalSimulator::ColorTopology()
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

	// big parameters, i.e. durations instead of rates
	double PWSteadyStateFunc(double selfDelta, double selfTau, double selfLambda, double selfReparationTime, std::vector<double> childrenPWs)
	{
		double mu = 1.0;
		for (int i = 0; i < childrenPWs.size(); i++)
			mu += childrenPWs[i];
		mu *= selfReparationTime + selfDelta / 2.0;

		return 1.0 / (1.0 + (selfTau / selfDelta) + (mu / selfLambda));
	}

	void ExperimentalSimulator::SetSNDeltas()
	{
		static constexpr int particleCount = 50;
		static constexpr double inertiaWeight = 0.5;
		static constexpr double cognitiveCoefficient = 1.5;
		static constexpr double socialCoefficient = 1.5;
		static constexpr int swarmBestChangeFinishThreshold = 200;


		std::vector<std::vector<int64_t>> children(m_SensorNodes.size());

		std::vector<std::vector<double>> particles;

		std::vector<std::vector<double>> particlesVelocity;

		std::vector<std::vector<double>> particlesBestCoords;

		std::vector<double> particlesBestValue;

		std::vector<double> swarmBestCoords;
		double swarmBestValue;

		int64_t maxLevel = -1;
		std::vector<double> failMean(m_SensorNodes.size());

		for (int i = 0; i < m_SensorNodes.size(); i++)
		{
			maxLevel = std::max(maxLevel, m_SensorNodes[i].m_Level);

			for (int j = 0; j < m_SensorNodes[i].m_FailureTimestamps.size(); j++)
			{
				if (j == 0)
					failMean[i] += m_SensorNodes[i].m_FailureTimestamps[j];
				else
					failMean[i] += m_SensorNodes[i].m_FailureTimestamps[j] - m_SensorNodes[i].m_FailureTimestamps[j - 1];
			}
			failMean[i] /= m_SensorNodes[i].m_FailureTimestamps.size();

			if (m_SensorNodes[i].m_CurrentParent == SensorNode::c_BaseStationIndex)
				continue;

			if (m_SensorNodes[i].m_CurrentParent == SensorNode::c_NoParentIndex ||
				m_SensorNodes[i].m_CurrentParent == SensorNode::c_InvalidIndex)
				throw std::runtime_error("An SN does not have any parents !");

			children[m_SensorNodes[i].m_CurrentParent].push_back(i);

		}

		auto CalculatePW = [&](std::vector<double> deltasIn)
		{
			std::vector<double> CWs(m_SensorNodes.size());

			for (int i = maxLevel; i >= 0; i--)
			{
				for (int j = 0; j < m_SensorNodes.size(); j++)
				{
					if (m_SensorNodes[j].m_Level != i)
						continue;

					std::vector<double> childrenCWs;
					for (int k = 0; k < children[j].size(); k++)
						//childrenPWs.push_back(PWs[children[j][k]]);
						childrenCWs.push_back(CWs[children[j][k]]);

					CWs[j] = PWSteadyStateFunc(deltasIn[j], m_SimulatorParameters.TransferTime,
						failMean[j], m_SimulatorParameters.RecoveryTime, childrenCWs);

					for (int k = 0; k < children[j].size(); k++)
						CWs[j] += CWs[children[j][k]];
				}
			}

			double BSTotal = 0.0;
			for (int i = 0; i < CWs.size(); i++)
			{
				if (m_SensorNodes[i].m_CurrentParent == SensorNode::c_BaseStationIndex)
					//BSTotal += PWs[i];
					BSTotal += CWs[i];
			}

			return BSTotal;
		};

		static constexpr double randomRangeHigh = 100000.0;
		static constexpr double randomRangeLow = 1.0;


		// mean = (A + B) / 2
		// stddev = sqrt((b-a) * (b-a) / 12)
		Distribution dist(DistributionType::Uniform, (randomRangeHigh + randomRangeLow) / 2.0,
			std::sqrt((randomRangeHigh - randomRangeLow) * (randomRangeHigh - randomRangeLow) / 12.0));

		Distribution dist01(DistributionType::Uniform, 1 / 2.0,
			std::sqrt(1 / 12.0));


		// particle count
		for (int i = 0; i < particleCount; i++)
		{
			std::vector<double> deltas;
			deltas.reserve(m_SensorNodes.size());
			for (int j = 0; j < m_SensorNodes.size(); j++)
				deltas.push_back(dist.GenerateRandomNumber());
			particles.push_back(deltas);
			particlesBestCoords.push_back(deltas);
			particlesBestValue.push_back(CalculatePW(deltas));

			std::vector<double> velos;
			velos.reserve(m_SensorNodes.size());
			for (int j = 0; j < m_SensorNodes.size(); j++)
			{
				double velo = dist.GenerateRandomNumber();
				if (s_RNG() % 2)
					velo *= -1;
				velos.push_back(velo);
			}
			particlesVelocity.push_back(velos);
		}


		{
			int bestIndex = -1;
			double bestValue = -1;
			for (int i = 0; i < particlesBestCoords.size(); i++)
			{
				if (particlesBestValue[i] > bestValue)
				{
					bestIndex = i;
					bestValue = particlesBestValue[i];
				}
			}

			swarmBestCoords = particlesBestCoords[bestIndex];
			swarmBestValue = bestValue;
		}

		int iterationsSinceLastSwarmBestChange = 0;
		int iteration = 0;

		while (iterationsSinceLastSwarmBestChange < swarmBestChangeFinishThreshold)
		{
			auto particlesTemp = particles;

			for (int particle = 0; particle < particleCount; particle++)
			{
				for (int dimension = 0; dimension < m_SensorNodes.size(); dimension++)
				{
					particlesVelocity[particle][dimension] =
						inertiaWeight * particlesVelocity[particle][dimension] +
						cognitiveCoefficient * dist01.GenerateRandomNumber() * (particlesBestCoords[particle][dimension] - particles[particle][dimension]) +
						socialCoefficient * dist01.GenerateRandomNumber() * (swarmBestCoords[dimension] - particles[particle][dimension]);
				}

				for (int dimension = 0; dimension < m_SensorNodes.size(); dimension++)
				{
					particlesTemp[particle][dimension] += particlesVelocity[particle][dimension];
					if (particlesTemp[particle][dimension] < 0.0)
						particlesTemp[particle][dimension] = 0.0;
					//if (particlesTemp[particle][dimension] > 100000.0)
					//	particlesTemp[particle][dimension] = 100000.0;
				}

				double temp = CalculatePW(particlesTemp[particle]);
				if (temp > particlesBestValue[particle])
				{
					particlesBestCoords[particle] = particlesTemp[particle];
					particlesBestValue[particle] = temp;

					if (temp > swarmBestValue)
					{
						swarmBestCoords = particlesTemp[particle];
						swarmBestValue = temp;
						iterationsSinceLastSwarmBestChange = -1;
					}

				}
			}

			particles = particlesTemp;

			iterationsSinceLastSwarmBestChange++;
			iteration++;

			//std::cout << "Iteration : " << iteration << '\n';
			//std::cout << "Delta = ( ";
			//for (int i = 0; i < swarmBestCoords.size(); i++)
			//	std::cout << swarmBestCoords[i] << ", ";
			//std::cout << " )\nPW = " << swarmBestValue << "\n--------------------------------------------------------------------------------------\n";
		}

		for (int i = 0; i < m_SensorNodes.size(); i++)
			m_SensorNodes[i].m_DeltaOpt = swarmBestCoords[i];
		m_SimulationResults.CWSNEfficiency = swarmBestValue / m_SensorNodes.size();

		std::cout << "Delta = ( ";
		for (int i = 0; i < swarmBestCoords.size(); i++)
			std::cout << swarmBestCoords[i] << ", ";
		std::cout << " )\nPW = " << swarmBestValue / m_SensorNodes.size() << "\n--------------------------------------------------------------------------------------\n";

		std::cout << "Done!\n";
	}


	void ExperimentalSimulator::Simulate()
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

					while (nextTime < currentTime)
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
					while (std::abs(nextTime - optimalTime) > std::abs(nextTime + m_SimulatorParameters.TransferTime * colorCount - optimalTime))
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
					m_SensorNodes[currentSN].m_CurrentPacketIterator = -1;
				}
			}
			else if (previousEvents[currentSN].State == WorkingState::Transfer)
			{
				if (currentState == WorkingState::Collection)
				{
					if (m_SensorNodes[currentSN].m_CurrentParent != SensorNode::c_BaseStationIndex)
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
					m_SensorNodes[currentSN].m_CurrentPacketIterator = -1;

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
					m_SensorNodes[currentSN].m_CurrentPacketIterator = -1;
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
		std::cout << "Transferred Total Duration = " << transferredTotalDuration << '\n';
		std::cout << "\n\n\n--------------------------------------------------------------------\n\n\n";

	}


	bool ExperimentalSimulator::IsDone(double currentTime)
	{
		return currentTime >= m_SimulatorParameters.TotalDurationToBeTransferred;
	}





}

