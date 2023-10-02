#include "datmo.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Datmo>());
  rclcpp::shutdown();

  return 0;
}
