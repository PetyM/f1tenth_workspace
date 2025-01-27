#include "cppagent/agentbase.h"

AgentBase::AgentBase()
{

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_client");
  rclcpp::shutdown();
  return 0;
}