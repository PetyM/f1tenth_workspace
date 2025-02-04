#ifndef PARAMETERS_H
#define PARAMETERS_H
 
namespace parameters
{


static constexpr double STEERING_MINIMUM = -0.4189;
static constexpr double STEERING_MAXIMUM = 0.4189;

static constexpr double STEERING_VELOCITY_MINIMUM = -3.2;
static constexpr double STEERING_VELOCITY_MAXIMUM = 3.2;


static constexpr double VELOCITY_MAXIMUM = 20.0;
static constexpr double VELOCITY_MINIMUM = -5.0;
static constexpr double ACCELERATION_MAXIMUM = 9.51;

static constexpr double VEHICLE_WIDTH = 0.31;
static constexpr double VEHICLE_LENGTH = 0.58;
static constexpr double VEHICLE_HEIGHT = 0.074;
static constexpr double VEHICLE_WEIGHT = 3.74;

static constexpr double AXLE_DISTANCE_FRONT = 0.15875;
static constexpr double AXLE_DISTANCE_REAR = 0.17145;

static constexpr double CORNERING_STIFFNESS_FRONT = 4.718;
static constexpr double CORNERING_STIFFNESS_REAR = 5.4562;

static constexpr double INERTIA = 0.04712;

};

#endif // PARAMETERS_H