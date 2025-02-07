#ifndef SAMPLINGAGENT_H
#define SAMPLINGAGENT_H

#include "cppagent/mapevaluatingagentbase.h"
#include "cppagent/dynamics.h"


class SamplingAgent : public MapEvaluatingAgentBase
{
    using BaseClass = MapEvaluatingAgentBase;
public:
    SamplingAgent();

protected:
    virtual Action plan();

private:
    static std::vector<Action> generateSamples(const State& state);

    using Trajectory = std::vector<State>;
    static std::vector<Trajectory> generateTrajectories(const std::vector<Action>& samples, const State& currentState);

    static constexpr Model MODEL = kinematicSingleTrackModel;
    static constexpr Integrator INTEGRATOR = rk4Integrator;

    static constexpr unsigned VELOCITY_SAMPLE_COUNT = 16;
    static constexpr unsigned STEERING_ANGLE_SAMPLE_COUNT = 16;
    
    static constexpr double VELOCITY_DIFFERENCE_MAXIMUM = 0.0;
    static constexpr double STEERING_ANGLE_DIFFERENCE_MAXIMUM = 0.0;

    static constexpr unsigned TRAJECTORY_POINT_COUNT = 20;
    static constexpr double TIME_DELTA = 0.1;

};



#endif // SAMPLINGAGENT_H