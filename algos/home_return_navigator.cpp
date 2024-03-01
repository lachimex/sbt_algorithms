#include "algo/task/home_return_navigator.hpp"

#include "state/vessel_state.hpp"

namespace algo {
namespace task {

HomeReturnNavigator::HomeReturnNavigator(rclcpp::Node* parentNode)
    : nodePtr{parentNode},
      internalStateID{HomeReturnStateID::ENTERED_TASK},
      gateFound{false},
      atHome{false} {}

static const auto LocalTaskIDMapper = [](const HomeReturnStateID stateID) {
  switch (stateID) {
    case HomeReturnStateID::ENTERED_TASK: {
      return "ENTERED_TASK";
      break;
    }
    case HomeReturnStateID::GOING_TO_HOME_WAYPOINT: {
      return "GOING_TO_HOME_WAYPOINT";
      break;
    }
    case HomeReturnStateID::STEER_TO_GATE: {
      return "STEER_TO_GATE";
      break;
    }
    case HomeReturnStateID::FINISHED_TASK: {
      return "FINISHED_TASK";
      break;
    }
    default:
      return "UNKNOWN";
      break;
  };
};

void HomeReturnNavigator::Run(state::VesselState& newState) {
  const auto lastState = internalStateID;
  internalStateID = stateMachine.GetNewState(lastState, newState);
  if (lastState != internalStateID) {
    RCLCPP_INFO(nodePtr->get_logger(), "TASK_STATE_CHG: [HomeReturn::%s -> HomeReturn::%s]",
                LocalTaskIDMapper(lastState), LocalTaskIDMapper(internalStateID));
  }

  RCLCPP_INFO(nodePtr->get_logger(), "HomeReturnNavigator");

  RCLCPP_INFO(nodePtr->get_logger(), "HomeReturnNavigator");

  switch (internalStateID) {
    case HomeReturnStateID::ENTERED_TASK: {
      RCLCPP_INFO(nodePtr->get_logger(), "HomeReturnNavigator::ENTERED_TASK");
      break;
    }
    case HomeReturnStateID::GOING_TO_HOME_WAYPOINT: {
      RCLCPP_INFO_STREAM(nodePtr->get_logger(), "HomeReturnNavigator::GOING_TO_HOME_WAYPOINT");
      HomeReturnNavigator::NavigateHomeWaypoint(newState);
      break;
    }
    case HomeReturnStateID::STEER_TO_GATE: {
      RCLCPP_INFO_STREAM(nodePtr->get_logger(), "HomeReturnNavigator::STEER_TO_GATE");
      HomeReturnNavigator::SteerToGate(newState);
      break;
    }
    case HomeReturnStateID::FINISHED_TASK: {
      RCLCPP_INFO(nodePtr->get_logger(), "HomeReturnNavigator::FINISHED_TASK");
      newState.controlValue.linear = 0.0;
      newState.controlValue.angular = 0.0;
      break;
    }
  }
  // RCLCPP_INFO(nodePtr->get_logger(), "INTERNAL STATE: %s", internalStateID);
}

// Heading from (0, 360) [degrees] -> (-180, 180) [degrees]
double HomeReturnNavigator::GetHeadingTo(double x0, double y0, double x1, double y1) {
  // returns alfa which is degree to turn alfa c <0,360>, <0,180> => turn right, else => turn left

  // setting our point P(x0.y0) in (0,0)
  y1 = y1 - y0;
  x1 = x1 - x0;

  // dividing by 0!
  if (x1 == 0) {
    if (y1 > 0)
      return 0;
    else
      return 180;
  }

  // phi takes values only from first quarter
  double phi = std::atan(std::fabs(y1 / x1)) * 180 / PI;

  if (x1 > 0 && y1 < 0) {
    // 4 quarter
    phi = 360 - phi;
  }
  else if (x1 < 0) {
    // 2 quarter
    if (y1 > 0) phi = 180 - phi;
    // 3 quarter
    else
      phi = 180 + phi;
  }

  return fmod(450 - phi, 360.0);  // alfa, fmod <=> remainder
}

void HomeReturnNavigator::ThrustHandler(state::VesselState& state, double speed, double angle) {
  state.controlValue.linear = speed;
  state.controlValue.angular = angle;
  RCLCPP_INFO(nodePtr->get_logger(), "Linear Velocity: %f,\nAngular Velocity: %f",
              state.controlValue.linear, state.controlValue.angular);
}

void HomeReturnNavigator::SearchForGate(state::VesselState& state) {
  std::vector<l2_interfaces::msg::Detection> blackBuoyDetection;
  for (const auto& det : state.smallBuoyDetections) {
    if (det.detection_type == l2_interfaces::msg::Detection::DETECTION_TYPE_BLACK) {
      blackBuoyDetection.push_back(det);
    }
  }
  if (blackBuoyDetection.size() >= 2) {
    gateFound = true;
  }
  else {
    gateFound = false;
  }
}

void HomeReturnNavigator::NavigateHomeWaypoint(state::VesselState& state) { 
  if (state.distanceToHome <= THRESHOLD_DISTANCE) {
    atHome = true;
    RCLCPP_INFO(nodePtr->get_logger(), "Distance to set point is smaller than THRESHOLD");
    return;
  }
  SearchForGate(state);
  if (gateFound) {
    SteerToGate(state);
    return;
  }
  double course_to_home = GetHeadingTo(state.currentPosition.x, state.currentPosition.y, 0,
                                       0);  //(0, 0) is our home coords
  RCLCPP_INFO(nodePtr->get_logger(), "Course to home: %f", course_to_home);
  double course_diff = state.currentHeading - course_to_home;
  if (course_diff > 180.0) {
    course_diff -= 360.0;
  }
  else if (course_diff < -180.0) {
    course_diff += 360.0;
  }
  double angle = course_diff / 180.0;
  ThrustHandler(state, SPEED, Clamp(-10.0 * angle * ANGLE_SCALE, -1.0, 1.0));
}

double HomeReturnNavigator::DetectionMidpoint(const l2_interfaces::msg::Detection& detection) {
  return ((detection.bbox_x1 - detection.bbox_x0) / 2.0) + detection.bbox_x0;
}

double HomeReturnNavigator::MiddlePointOfTheGate(double midPointLeft, double midPointRight) {
  return ((midPointRight - midPointLeft) / 2.0) + midPointLeft;
}

void HomeReturnNavigator::SteerToGate(state::VesselState& state) {
  std::vector<l2_interfaces::msg::Detection> blackBuoyDetections;
  for (const auto& det : state.smallBuoyDetections) {
    if (det.detection_type == l2_interfaces::msg::Detection::DETECTION_TYPE_BLACK) {
      blackBuoyDetections.push_back(det);
    }
  }
  std::vector<double> detectionsMidpoints;
  for (const auto& det : blackBuoyDetections) {
    detectionsMidpoints.push_back(DetectionMidpoint(det));
  }
  double gateMid;
  double midPointLeft = 640.0;  // max value of left most midpoint
  double midPointRight = 0.0;   // min value of right most midpoint
  for (double midpoint : detectionsMidpoints) {
    if (midpoint < midPointLeft) {
      midPointLeft = midpoint;
    }
    if (midpoint > midPointRight) {
      midPointRight = midpoint;
    }
  }
  gateMid = MiddlePointOfTheGate(midPointLeft, midPointRight);
  double angle = gateMid/320.0 - 1.0;
  ThrustHandler(state, SPEED, angle * ANGLE_SCALE);
}

}  // namespace task
}  // namespace algo
