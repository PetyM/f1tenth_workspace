#ifndef STATE_H
#define STATE_H

struct State
{
    double positionX;
    double positionY;
    double theta;
    double velocity;
    double steeringAngle;

    State operator+(const State& other) const
    {
        return State{
            .positionX = positionX + other.positionX,
            .positionY = positionY + other.positionY,
            .theta = theta + other.theta,
            .velocity = velocity + other.velocity,
            .steeringAngle = steeringAngle + other.steeringAngle
        };
    }

    State operator*(double dt) const
    {
        return State{
            .positionX = positionX *  dt,
            .positionY = positionY * dt,
            .theta = theta * dt,
            .velocity = velocity * dt,
            .steeringAngle = steeringAngle * dt
        };
    }
};


#endif // STATE_H