#include "cppagent/samplingagent.h"

#include <execution>

SamplingAgent::SamplingAgent()
    : BaseClass()
{

}

SamplingAgent::Action SamplingAgent::plan()
{
    const State state = getState();
    const std::vector<Action> samples = generateSamples(state);
    const std::vector<Trajectory> trajectories = generateTrajectories(samples);

    // TODO: evaluate trajectories and pick best

    return samples.at(0);
}

std::vector<SamplingAgent::Action> SamplingAgent::generateSamples(const SamplingAgent::State& state)
{
    std::vector<Action> samples = {};
    samples.reserve(VELOCITY_SAMPLE_COUNT * STEERING_ANGLE_SAMPLE_COUNT);
    
    const double velocityMinimum = std::max(state.velocity - VELOCITY_DIFFERENCE_MAXIMUM, VELOCITY_MINIMUM);
    const double velocityMaximum = std::min(state.velocity + VELOCITY_DIFFERENCE_MAXIMUM, VELOCITY_MAXIMUM);
    const double velocityStep = (velocityMaximum - velocityMinimum) / static_cast<double>(VELOCITY_SAMPLE_COUNT);

    const double steeringAngleMinimum = std::max(state.steeringAngle - STEERING_ANGLE_DIFFERENCE_MAXIMUM, STEERING_ANGLE_MINIMUM);
    const double steeringAngleMaximum = std::min(state.steeringAngle + STEERING_ANGLE_DIFFERENCE_MAXIMUM, STEERING_ANGLE_MAXIMUM);
    const double steeringAngleStep = (steeringAngleMaximum - steeringAngleMinimum) / static_cast<double>(STEERING_ANGLE_SAMPLE_COUNT);

    for (int i = 0; i < VELOCITY_SAMPLE_COUNT; ++i)
    {
        for (int j = 0; j < STEERING_ANGLE_SAMPLE_COUNT; ++j)
        {
            samples.push_back(Action {
                                .velocity = velocityMinimum + (i * velocityStep),
                                .steeringAngle = steeringAngleMinimum + (j * steeringAngleStep)
                              });
        }
    }

    return samples;
}

std::vector<SamplingAgent::Trajectory> SamplingAgent::generateTrajectories(const std::vector<Action>& samples)
{
    std::vector<Trajectory> trajectories = {};
    trajectories.reserve(samples.size());

    static constexpr auto generateTrajectory = [](const Action& sample) -> Trajectory
    {
        Trajectory poses = {};
        poses.reserve(TRAJECTORY_POINT_COUNT);

        for (int i = 0; i < TRAJECTORY_POINT_COUNT; ++i)
        {
            // TODO: dynamics
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