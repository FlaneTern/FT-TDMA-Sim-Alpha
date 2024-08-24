
#include "PCH.h"
#include "Problem.h"

namespace FT_TDMA_Sim
{
	Problem::Problem()
	{
		static int64_t currentProblemID = 0;

		currentProblemID++;
		m_ProblemID = currentProblemID;

		GenerateSNs();
		GenerateSNsPost();
		GenerateFailures();
		GenerateFailuresPost();
	}

	void Problem::Run()
	{
		if (m_HasRun)
			throw std::runtime_error("This problem has been ran !");
		m_HasRun = true;

		for (int i = 0; i < m_Simulators.size(); i++)
		{
			m_Simulators[i]->Run(m_SensorNodes);
		}
	}

	void Problem::GenerateSNs()
	{

		static constexpr uint64_t TotalSNCount = 100;

		// mean = (A + B) / 2
		// stddev = sqrt((b-a) * (b-a) / 12)
		Distribution dist(DistributionType::Uniform, 1000.0 / 2.0,
			std::sqrt(1000.0 * 1000.0 / 12.0));


		for (int SNCount = 0; SNCount < TotalSNCount; SNCount++)
		{
			double xPos = dist.GenerateRandomNumber() * (s_RNG() % 2 ? -1 : 1);
			double yPos = dist.GenerateRandomNumber() * (s_RNG() % 2 ? -1 : 1);

			SensorNode sn;
			sn.m_ID = (int64_t)SNCount;
			sn.m_Position = { xPos, yPos };

			m_SensorNodes.push_back(sn);
		}


	}

	void Problem::GenerateFailures()
	{
		static constexpr double failGenerationDuration = 3600 * 24 * 90;

		Distribution uniformDist(DistributionType::Exponential, 3600 * 24, 3600 * 24);

		for (int i = 0; i < m_SensorNodes.size(); i++)
		{
			double currentTime = 0;
			double timeToNextFailure;
			while (currentTime < failGenerationDuration)
			{
				timeToNextFailure = uniformDist.GenerateRandomNumber();
				currentTime += timeToNextFailure;
				// LOOK AT THIS
				//sr.Failures.push_back({ (int64_t)i, currentTime });
				m_SensorNodes[i].m_FailureTimestamps.push_back(currentTime);
			}
		}
	}

	void Problem::GenerateSNsPost()
	{

	}

	void Problem::GenerateFailuresPost()
	{

	}
}