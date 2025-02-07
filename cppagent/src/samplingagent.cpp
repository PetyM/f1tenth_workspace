#include "cppagent/samplingagent.h"

#include "cppagent/parameters.h"
#include "cppagent/fuzzycompare.h"

#include <execution>

SamplingAgent::SamplingAgent()
    : BaseClass()
{

}

Action SamplingAgent::plan()
{
    const State state = getState();
    const std::vector<Action> samples = generateSamples(state);
    const std::vector<Trajectory> trajectories = generateTrajectories(samples, state);

    // TODO: evaluate trajectories and pick best

    return samples.at(0);
}

std::vector<Action> SamplingAgent::generateSamples(const State& state)
{
    std::vector<Action> samples = {};
    samples.reserve(VELOCITY_SAMPLE_COUNT * STEERING_ANGLE_SAMPLE_COUNT);
    
    const double accelerationMinimum = nearZero(state.velocity) ? 0 : -parameters::ACCELERATION_MAXIMUM;
    const double accelerationMaximum = equal(state.velocity, parameters::VELOCITY_MAXIMUM) ? 0 : parameters::ACCELERATION_MAXIMUM;
    const double accelerationStep = (accelerationMaximum - accelerationMinimum) / static_cast<double>(VELOCITY_SAMPLE_COUNT);

    const double steeringSpeedMinimum = equal(state.steeringAngle, parameters::STEERING_MINIMUM) ? 0 : parameters::STEERING_VELOCITY_MINIMUM;
    const double steeringSpeedMaximum = equal(state.steeringAngle, parameters::STEERING_MAXIMUM) ? 0 : parameters::STEERING_VELOCITY_MAXIMUM;
    const double steeringSpeedStep = (steeringSpeedMaximum - steeringSpeedMinimum) / static_cast<double>(STEERING_ANGLE_SAMPLE_COUNT);

    for (int i = 0; i < VELOCITY_SAMPLE_COUNT; ++i)
    {
        for (int j = 0; j < STEERING_ANGLE_SAMPLE_COUNT; ++j)
        {
            samples.push_back(Action {
                                .acceleration = accelerationMinimum + (i * accelerationStep),
                                .steeringVelocity = steeringSpeedMinimum + (j * steeringSpeedStep)
                              });
        }
    }

    return samples;
}

std::vector<SamplingAgent::Trajectory> SamplingAgent::generateTrajectories(const std::vector<Action>& samples, const State& currentState)
{
    std::vector<Trajectory> trajectories = {};
    trajectories.reserve(samples.size());

    const auto generateTrajectory = [currentState](const Action& sample) -> Trajectory
    {
        Trajectory poses = {currentState};
        poses.reserve(TRAJECTORY_POINT_COUNT);

        for (int i = 1; i < TRAJECTORY_POINT_COUNT; ++i)
        {
            poses[i] = INTEGRATOR(poses[i - 1], sample, TIME_DELTA, MODEL);
        }
        return poses;
    };

    std::transform(std::execution::par_unseq, samples.cbegin(), samples.cend(), trajectories.begin(), generateTrajectory);
    return trajectories;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SamplingAgent>());
    rclcpp::shutdown();
    return 0;
}