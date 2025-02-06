#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "cppagent/action.h"
#include "cppagent/state.h"

#include <functional>

Action constrainAction(const State& state, const Action& action);

using Dynamics = State (*) (const State&, const Action&);
State kinematicSingleTrackModel(const State& state, const Action& action);

using Integrator = State (*) (const State&, const Action&, double, Dynamics);
State rk4Integrator(const State& state, const Action& action, double dt, Dynamics dynamics);

#endif // DYNAMICS_H