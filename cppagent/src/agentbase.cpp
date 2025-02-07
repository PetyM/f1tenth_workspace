#include "cppagent/agentbase.h"

#include "cppagent/utils.h"

#include <format>


AgentBase::AgentBase()
    : BaseClass("agent")
{
    declare_parameter("agent_namespace", "ego_racecar");
    m_agentNamespace = get_parameter("agent_namespace").as_string();
    declare_parameter("opponent_namespace", "opp_racecar");
    m_opponentNamespace = get_parameter("opponent_namespace").as_string();
    declare_parameter("state_topic", "state");
    m_stateTopic = get_parameter("state_topic").as_string();
    declare_parameter("drive_topic", "drive");
    m_driveTopic = get_parameter("drive_topic").as_string();
    declare_parameter("velocity_gain", 1.0);
    m_velocityGain = get_parameter("velocity_gain").as_double();
    declare_parameter("opponent_present", false);
    m_opponentPresent = get_parameter("opponent_present").as_bool();

    const std::string driveTopic = std::format("{}/{}", m_agentNamespace, m_driveTopic);
    m_drivePublisher = create_publisher<AckermannDriveStamped>(driveTopic, 1);

    const std::string stateTopic = std::format("{}/{}", m_agentNamespace, m_stateTopic);
    const auto stateCallback = [this](Odometry::SharedPtr state)
    {
        EulerAngles e = quaternionToEuler(state->pose.pose.orientation);
        m_state.set(State{
            .positionX = state->pose.pose.position.x,
            .positionY = state->pose.pose.position.y,
            .theta = e.yaw,
            .velocity = state->twist.twist.linear.x,
            .steeringAngle = state->twist.twist.angular.z
        });
    };
    m_stateSubscription = create_subscription<Odometry>(stateTopic, 1, stateCallback);

    const std::string opponentStateTopic = std::format("{}/{}", m_opponentNamespace, m_stateTopic);
    const auto opponentStateCallback = [this](Odometry::SharedPtr state)
    {
        EulerAngles e = quaternionToEuler(state->pose.pose.orientation);
        m_opponentState.set(State{
            .positionX = state->pose.pose.position.x,
            .positionY = state->pose.pose.position.y,
            .theta = e.yaw,
            .velocity = state->twist.twist.linear.x,
            .steeringAngle = state->twist.twist.angular.z
        });
    };
    m_opponentStateSubscription = create_subscription<Odometry>(opponentStateTopic, 1, opponentStateCallback);

    m_timer = create_wall_timer(std::chrono::seconds(1), [this](){ updateControl(); });
}

State AgentBase::getState() const
{
    return m_state.get();
}

State AgentBase::getOpponentState() const
{
    return m_opponentState.get();
}

void AgentBase::updateControl()
{
    auto action = plan();

    auto message = AckermannDriveStamped();
    message.header.stamp = get_clock()->now();
    message.drive.acceleration = action.acceleration;
    message.drive.steering_angle_velocity = action.steeringVelocity;
    m_drivePublisher->publish(message);
}

const std::string& AgentBase::agentNamespace() const
{
    return m_agentNamespace;
}

const std::string& AgentBase::opponentNamespace() const
{
    return m_opponentNamespace;
}

const std::string& AgentBase::stateTopic() const
{
    return m_stateTopic;
}

const std::string& AgentBase::driveTopic() const
{
    return m_driveTopic;
}

double AgentBase::velocityGain() const
{
    return m_velocityGain;
}

bool AgentBase::opponentPresent() const
{
    return m_opponentPresent;
}
