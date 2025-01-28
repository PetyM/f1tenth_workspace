#ifndef AGENTBASE_H
#define AGENTBASE_H

#include "cppagent/lockedvalue.h"

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "rclcpp/rclcpp.hpp"


class AgentBase : public rclcpp::Node
{
    using BaseClass = rclcpp::Node;
public:
    AgentBase();

protected:
    using State = std::vector<double>;
    State state() const;
    State opponentState() const;

    virtual void updateControl();
    
    struct Action
    {
        double velocity;
        double steeringAngle;
    };
    virtual Action plan() = 0;

    const std::string& agentNamespace() const;
    const std::string& opponentNamespace() const;
    const std::string& stateTopic() const;
    const std::string& driveTopic() const;
    double velocityGain() const;
    bool opponentPresent() const;

private:
    using AckermannDriveStamped = ackermann_msgs::msg::AckermannDriveStamped;
    rclcpp::Publisher<AckermannDriveStamped>::SharedPtr m_drivePublisher;

    using Float64MultiArray = std_msgs::msg::Float64MultiArray;
    rclcpp::Subscription<Float64MultiArray>::SharedPtr m_stateSubscription;
    rclcpp::Subscription<Float64MultiArray>::SharedPtr m_opponentStateSubscription;

    rclcpp::TimerBase::SharedPtr m_timer;

    LockedValue<State> m_state;
    LockedValue<State> m_opponentState;

    std::string m_agentNamespace;
    std::string m_opponentNamespace;
    std::string m_stateTopic;
    std::string m_driveTopic;
    double m_velocityGain;
    bool m_opponentPresent;
};

#endif // AGENTBASE_H