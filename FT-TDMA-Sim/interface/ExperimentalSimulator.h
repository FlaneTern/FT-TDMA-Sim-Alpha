
#pragma once
#include "Simulator.h"

namespace FT_TDMA_Sim
{
	class ExperimentalSimulator : public Simulator
	{
	public:
		ExperimentalSimulator(SimulatorParameters sp);
	protected:

		virtual void ConstructTopology() override;
		virtual void ColorTopology() override;
		virtual void SetSNDeltas() override;

		virtual bool IsDone(double currentTime) override;


		virtual void Simulate() override;

		//virtual void Reroute() override;
	};
}