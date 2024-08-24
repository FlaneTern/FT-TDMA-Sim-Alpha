#include "PCH.h"

#include "InterfaceExample.h"
#include "ExperimentalSimulator.h"
#include "Problem.h"

int main()
{

	std::shared_ptr<FT_TDMA_Sim::Problem> problem = std::make_shared<FT_TDMA_Sim::Problem>();

#pragma region
	std::vector<double> totalDurationToBeTransferreds =
	{
		3600 * 24 * 90,
	};

	std::vector<double> transmissionRanges =
	{
		10000,
	};


	std::vector<double> interferenceRanges =
	{
		100,
	};

	std::vector<double> transferTimes =
	{
		30.0 * 1,
	};

	std::vector<double> recoveryTimes =
	{
		3600.0 * 12,
	};

	std::vector<double> energyRateTransfers =
	{
		8.0,
	};

	std::vector<double> energyRateWorkings =
	{
		1.0,
	};

	FT_TDMA_Sim::SimulatorParameterGrid spg =
	{
		totalDurationToBeTransferreds,
		transferTimes,
		recoveryTimes,
		energyRateWorkings,
		energyRateTransfers,
		transmissionRanges,
		interferenceRanges
	};

	std::vector<std::shared_ptr<FT_TDMA_Sim::Simulator>> exampleSimulators = FT_TDMA_Sim::Simulator::CreateSimulator<FT_TDMA_Sim::ExampleSimulator>(spg);
	std::vector<std::shared_ptr<FT_TDMA_Sim::Simulator>> experimentalSimulators = FT_TDMA_Sim::Simulator::CreateSimulator<FT_TDMA_Sim::ExperimentalSimulator>(spg);
	problem->AddSimulator(exampleSimulators);
	problem->AddSimulator(experimentalSimulators);

	problem->Run();
#pragma endregion

	return 0;
}