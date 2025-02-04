#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "cppagent/action.h"
#include "cppagent/state.h"

#include <functional>

using Model = State (*) (const State&, const Action&);
State kinematicModel(const State& state, const Action& action);
State singleTrackModel(const State& state, const Action& action);
State twinTrackModel(const State& state, const Action& action);

using Integrator = State (*) (const State&, const Action&, double, Model);
State rk4Integrator(const State& state, const Action& action, double dt, Model model);

#endif // DYNAMICS_H