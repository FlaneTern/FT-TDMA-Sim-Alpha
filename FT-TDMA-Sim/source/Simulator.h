#pragma once
#include "Distribution.h"
#include "SensorNode.h"
#include "DataInterface.h"

namespace FT_TDMA_Sim
{

	struct Failure
	{
		int64_t SNID;
		double Timestamp;
	};

	/// <summary>
	/// Sensor node state at a certain time interval
	/// </summary>
	struct SimulationInterval
	{
		WorkingState State;
		double StartTime;
		double EndTime;
	};			

	struct SimulationResults
	{
		double ActualTotalDuration = 0;
		int64_t FinalFailureIndex = 0;
		double CWSNEfficiency = 0;

		std::vector<Failure> Failures;
	};

	/// <summary>
	/// Simulation hyperparameters
	/// </summary>
	struct SimulatorParameters
	{
		double TotalDurationToBeTransferred = -1;
		double TransferTime = -1;
		double RecoveryTime = -1;

		double EnergyRateWorking;
		double EnergyRateTransfer;

		double TransmissionRange;
		double InterferenceRange;
	};

	struct SimulatorParameterGrid
	{
		std::vector<double> TotalDurationToBeTransferred;
		std::vector<double> TransferTime;
		std::vector<double> RecoveryTime;

		std::vector<double> EnergyRateWorking;
		std::vector<double> EnergyRateTransfer;

		std::vector<double> TransmissionRange;
		std::vector<double> InterferenceRange;
	};


	class Simulator
	{
	public:

		void Run(const std::vector<SensorNode>& SNs);


		inline int64_t GetSimulationID() const { return m_SimulationID; }
		inline SimulatorParameters GetSimulatorParameters() const { return m_SimulatorParameters; }

		I_SimulatorData i_SimulationData;

		template<typename T>
		static inline std::shared_ptr<Simulator> CreateSimulator(SimulatorParameters sp) { return T(sp); }

		template<typename T>
		static std::vector<std::shared_ptr<Simulator>> CreateSimulator(SimulatorParameterGrid spg)
		{
			std::vector<std::shared_ptr<Simulator>> simulators;

			for (auto& recoveryTime : spg.RecoveryTime)
			{
				for (auto& transferTime : spg.TransferTime)
				{
					for (auto& totalDurationToBeTransferred : spg.TotalDurationToBeTransferred)
					{
						for (auto& energyRateWorking : spg.EnergyRateWorking)
						{
							for (int energyRateTransfer : spg.EnergyRateTransfer)
							{
								for (auto& transmissionRange : spg.TransmissionRange)
								{
									for (auto& interferenceRange : spg.InterferenceRange)
									{
										FT_TDMA_Sim::SimulatorParameters sp =
										{
											totalDurationToBeTransferred,
											transferTime,
											recoveryTime,
											energyRateWorking,
											energyRateTransfer,
											transmissionRange,
											interferenceRange
										};

										simulators.push_back(std::make_shared<T>(sp));
									}
								}
							}
						}
					}
				}
			}

			return simulators;
		}

	protected:
		Simulator() = delete;
		Simulator(SimulatorParameters sp);

		virtual void Simulate();

		virtual void ConstructTopology();
		virtual void ColorTopology();
		virtual void SetSNDeltas();

		virtual bool IsDone(double currentTime);

		void ConstructTopologyPost();
		void ColorTopologyPost();
		void SetSNDeltasPost();

		virtual void Reroute();

		int64_t m_SimulationID;

		std::vector<SensorNode> m_SensorNodes;

		SimulatorParameters m_SimulatorParameters;

		SimulationResults m_SimulationResults;
	private:

		//static std::vector<SimulationSummaryData> s_Summary;

	};


}