#pragma once
#include "Simulator.h"

namespace FT_TDMA_Sim
{
	class Problem
	{
	public:
		Problem();

		inline void AddSimulator(std::shared_ptr<Simulator> simulator) { m_Simulators.push_back(simulator); }
		inline void AddSimulator(std::vector<std::shared_ptr<Simulator>> simulators) { for(auto& simulator : simulators) m_Simulators.push_back(simulator); }

		void Run();

	protected:
		virtual void GenerateSNs();
		virtual void GenerateFailures();

		void GenerateSNsPost();
		void GenerateFailuresPost();

		int64_t m_ProblemID;

		std::vector<SensorNode> m_SensorNodes;

		std::vector<std::shared_ptr<Simulator>> m_Simulators;

	private:
		bool m_HasRun = false;
	};
}