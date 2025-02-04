
#include "cppagent/dynamics.h"

#include "cppagent/parameters.h"

State kinematicModel(const State& state, const Action& action)
{
    return {};
}

State singleTrackModel(const State& state, const Action& action)
{
    return {};
}

State twinTrackModel(const State& state, const Action& action)
{
    return {};
}

State rk4Integrator(const State& state, const Action& action, double dt, Model model)
{
    const State d1 = model(state, action);
    const State s1 = state + d1 * (dt / 2.0);

    const State d2 = model(s1, action);
    const State s2 = state + d2 * (dt / 2.0);

    const State d3 = model(s2, action);
    const State s3 = state + d3 * dt;

    const State d4 = model(s3, action);

    return state + ((d1 + (d2 * 2.0) + (d3 * 2.0) + d4) * (dt / 6.0));
}
