#ifndef SAMPLINGAGENT_H
#define SAMPLINGAGENT_H

#include "cppagent/mapevaluatingagentbase.h"


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
    static std::vector<Trajectory> generateTrajectories(const std::vector<Action>& samples);


    static constexpr unsigned VELOCITY_SAMPLE_COUNT = 16;
    static constexpr unsigned STEERING_ANGLE_SAMPLE_COUNT = 16;
    
    static constexpr double VELOCITY_MINIMUM = 0.0;
    static constexpr double VELOCITY_MAXIMUM = 0.0;
    static constexpr double VELOCITY_DIFFERENCE_MAXIMUM = 0.0;

    static constexpr double STEERING_ANGLE_MINIMUM = 0.0;
    static constexpr double STEERING_ANGLE_MAXIMUM = 0.0;
    static constexpr double STEERING_ANGLE_DIFFERENCE_MAXIMUM = 0.0;

    static constexpr unsigned TRAJECTORY_POINT_COUNT = 20;

};



#endif // SAMPLINGAGENT_H