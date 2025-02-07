
#include "cppagent/dynamics.h"

#include "cppagent/parameters.h"

#include <cmath>

Action constrainAction(const State& state, const Action& action)
{
    const double maximumAcceleration = state.velocity > parameters::WHEEL_SWITCH_VELOCITY 
                                        ? parameters::ACCELERATION_MAXIMUM * (parameters::WHEEL_SWITCH_VELOCITY / state.velocity)
                                        : parameters::ACCELERATION_MAXIMUM;
    return Action{
        .acceleration = std::max(std::min({action.acceleration, 
                                           maximumAcceleration,
                                           parameters::ACCELERATION_MAXIMUM * (parameters::WHEEL_SWITCH_VELOCITY / state.velocity)}),
                                 -parameters::ACCELERATION_MAXIMUM),
        .steeringVelocity = std::max(std::min(action.steeringVelocity, 
                                              parameters::STEERING_VELOCITY_MAXIMUM), 
                                     parameters::STEERING_VELOCITY_MINIMUM)
    };
}

State kinematicSingleTrackModel(const State& state, const Action& action)
{
    const Action constrainedAction = constrainAction(state, action);
    return State{
        .positionX = state.velocity * cos(state.theta),
        .positionY = state.velocity * sin(state.theta),
        .theta = (state.velocity * tan(state.steeringAngle)) / parameters::AXLE_DISTANCE,
        .velocity = constrainedAction.acceleration,
        .steeringAngle = constrainedAction.steeringVelocity
    };
}

State rk4Integrator(const State& state, const Action& action, double dt, Model dynamics)
{
    const State d1 = dynamics(state, action);
    const State s1 = state + d1 * (dt / 2.0);

    const State d2 = dynamics(s1, action);
    const State s2 = state + d2 * (dt / 2.0);

    const State d3 = dynamics(s2, action);
    const State s3 = state + d3 * dt;

    const State d4 = dynamics(s3, action);

    return state + ((d1 + (d2 * 2.0) + (d3 * 2.0) + d4) * (dt / 6.0));
}
