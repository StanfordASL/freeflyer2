#include "ff_control/linear_ctrl.hpp"

namespace ff {

LinearController::LinearController()
  : rclcpp::Node("linear_ctrl_node"),
    WrenchController() {

}

} // namespace ff
