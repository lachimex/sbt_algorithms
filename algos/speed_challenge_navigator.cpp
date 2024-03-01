#include "algo/task/speed_challenge_navigator.hpp"

#include "state/vessel_state.hpp"

namespace algo {
namespace task {

SpeedChallengeNavigator::SpeedChallengeNavigator(rclcpp::Node* parentNode)
    : nodePtr{parentNode},
      internalStateID{SpeedChallengeStateID::ENTERED_TASK},
      ifCloseToTheYellow{false},
      ifCloseToTheGate{false},
      gateCrossedStart{false},
      gateCrossedEnd{false},
      buoyManouvered{false},
      goingRoundBlue{false},
      gateFound{false} {}

static const auto LocalTaskIDMapper = [](const SpeedChallengeStateID taskID) {
  switch (taskID) {
    case SpeedChallengeStateID::ENTERED_TASK: {
      return "ENTERED_TASK";
      break;
    }
    case SpeedChallengeStateID::STEER_TO_GATE: {
      return "STEER_TO_GATE";
      break;
    }
    case SpeedChallengeStateID::GO_TO_THE_BUOY: {
      return "GO_TO_THE_BUOY";
      break;
    }
    case SpeedChallengeStateID::MANOUVER_AROUND_BUOY: {
      return "MANOUVER_AROUND_BUOY";
      break;
    }
    case SpeedChallengeStateID::STEER_TO_GATE_RETURN: {
      return "STEER_TO_GATE_RETURN";
      break;
    }
    case SpeedChallengeStateID::FINISHED_TASK: {
      return "FINISHED_TASK";
      break;
    }
    default:
      return "UNKNOWN";
  }
};

void SpeedChallengeNavigator::Run(state::VesselState& newState) {
  const auto lastState = internalStateID;
  internalStateID = stateMachine.GetNewState(lastState, newState);
  if (lastState != internalStateID) {
    RCLCPP_INFO(nodePtr->get_logger(), "TASK_STATE_CHG: [SpeedChallenge::%s -> SpeedChallenge::%s]",
                LocalTaskIDMapper(lastState), LocalTaskIDMapper(internalStateID));
  }
  switch (internalStateID) {
    case SpeedChallengeStateID::ENTERED_TASK: {
      StationKeep(newState);
      break;
    }
    case SpeedChallengeStateID::STEER_TO_GATE: {
      SteerToGate(newState);
      break;
    }
    case SpeedChallengeStateID::GO_TO_THE_BUOY: {
      SteerAndManouverToYellowBuoy(newState);
      break;
    }
    case SpeedChallengeStateID::MANOUVER_AROUND_BUOY: {  // think if manouvering needs to be in
                                                         // other function
      SteerAndManouverToYellowBuoy(newState);
      break;
    }
    case SpeedChallengeStateID::STEER_TO_GATE_RETURN: {
      SteerToGate(newState);
      break;
    }
    case SpeedChallengeStateID::FINISHED_TASK: {
      ThrustHandler(newState, 0.1, 0.0);
      newState.currentTask = state::TaskID::COLLECTION_OCT;
    }
  }
}

void SpeedChallengeNavigator::ThrustHandler(state::VesselState& state, double speed, double angle) {
  state.controlValue.linear = speed;
  state.controlValue.angular = angle;
}

namespace {
double Area(const double x, const double y) { return x * y; }
}  // namespace

double SpeedChallengeNavigator::DetectionMidpoint(const l2_interfaces::msg::Detection& detection) {
  return ((detection.bbox_x1 - detection.bbox_x0) / 2.0) + detection.bbox_x0;
}

double SpeedChallengeNavigator::MiddlePointOfTheGate(double midPointRed, double midPointGreen) {
  return ((midPointGreen - midPointRed) / 2.0) + midPointRed;
}

void SpeedChallengeNavigator::ThrustHandlerForTurning(state::VesselState& state, TurnID turnID){
  if (turnID == TurnID::GO_STRAIGHT) {
    RCLCPP_INFO(nodePtr->get_logger(), "GO STRAIGHT");
    state.controlValue.linear = 0.25;
    state.controlValue.angular = 0.0;
  }
  else if (turnID == TurnID::TURN_RIGHT_SLOW) {
    RCLCPP_INFO(nodePtr->get_logger(), "TURN RIGHT SLOW");
    state.controlValue.linear = 0.25;
    state.controlValue.angular = 0.1;
  }
  else if (turnID == TurnID::TURN_RIGHT_FAST) {
    RCLCPP_INFO(nodePtr->get_logger(), "TURN RIGHT FAST");
    state.controlValue.linear = 0.25;
    state.controlValue.angular = 0.4;
  }
  else if (turnID == TurnID::TURN_LEFT_SLOW) {
    RCLCPP_INFO(nodePtr->get_logger(), "TURN LEFT SLOW");
    state.controlValue.linear = 0.25;
    state.controlValue.angular = -0.1;
  }
  else if (turnID == TurnID::TURN_LEFT_FAST) {
    RCLCPP_INFO(nodePtr->get_logger(), "TURN LEFT FAST");
    state.controlValue.linear = 0.25;
    state.controlValue.angular = -0.4;
  }
  RCLCPP_INFO(nodePtr->get_logger(), "Linear Velocity: %f,\nAngular Velocity: %f",
               state.controlValue.linear, state.controlValue.angular);
}

bool SpeedChallengeNavigator::FindGate(state::VesselState& state) {
  std::vector<l2_interfaces::msg::Detection> redBuoyDetections;
  std::vector<l2_interfaces::msg::Detection> greenBuoyDetections;

  for (const auto& det : state.bigBuoyDetections) {
    if (det.detection_type == l2_interfaces::msg::Detection::DETECTION_TYPE_RED) {
      redBuoyDetections.push_back(det);
    }
    if (det.detection_type == l2_interfaces::msg::Detection::DETECTION_TYPE_GREEN) {
      greenBuoyDetections.push_back(det);
    }
  }
  if (redBuoyDetections.size() > 0 && greenBuoyDetections.size() > 0) {
    return true;
  }
  else {
    return false;
  }
}

void SpeedChallengeNavigator::StationKeep(state::VesselState& state) {
  state.controlValue.linear = 0.0;
  state.controlValue.angular = 0.2;
  if (FindGate(state)) {
    gateFound = true;
  }
}

void SpeedChallengeNavigator::SteerToMidpoint(double midpoint, state::VesselState& state) {
  double angle = midpoint / 320.0 - 1.0;
  ThrustHandler(state, SPEED, Clamp(angle * ANGLE_SCALE, -1.0, 1.0));
}

void SpeedChallengeNavigator::SteerAndManouverToYellowBuoy(state::VesselState& state) {
  if (gateFound) {
    gateFound = false;  // need to reset it for coming back
  }
  std::optional<l2_interfaces::msg::Detection> yellowBuoyDetection;
  std::optional<l2_interfaces::msg::Detection> blueBuoyDetection;  // colission avoidance of blue
  for (const auto& det : state.bigBuoyDetections) {
    if (det.detection_type == l2_interfaces::msg::Detection::DETECTION_TYPE_YELLOW) {
      yellowBuoyDetection = det;
    }
    if (det.detection_type == l2_interfaces::msg::Detection::DETECTION_TYPE_BLUE) {
      blueBuoyDetection = det;
    }
  }
  RCLCPP_INFO(nodePtr->get_logger(), "yellowBuoyDetection: %s, blueBuoyDetection: %s",
              yellowBuoyDetection.has_value() ? "true" : "false",
              blueBuoyDetection.has_value() ? "true" : "false");

  if (!yellowBuoyDetection.has_value() && !ifCloseToTheYellow) {
    ThrustHandler(state, 0.0, 0.2);
    RCLCPP_INFO(nodePtr->get_logger(), "GETTING CLOSER TO DETECTIONS");
    return;
  }

  if (ifCloseToTheYellow && FindGate(state)) {
    buoyManouvered = true;
    counterForBlue = 0;
  }
  // steering to yellow buoy
  if (!ifCloseToTheYellow && yellowBuoyDetection.has_value()) {
    RCLCPP_INFO(nodePtr->get_logger(), "STEERING TO YELLOW BUOY");
    double middleOfBuoy = DetectionMidpoint(yellowBuoyDetection.value());
    SteerToMidpoint(middleOfBuoy, state);
  }
  // we are close to the yellow -> we try to keep right side of yellow buoy in the 1/4 of view on
  // the left side
  if (yellowBuoyDetection.has_value() && yellowBuoyDetection.value().area >= THRESHOLD_AREA) {
    // yellowBuoyDetection.value().distance <= THRESHOLD_DISTANCE) {
    // changed for sim
    counterForYellow++;
    if (counterForYellow >= DESIRED_NUMBER_OF_COUNTS || ifCloseToTheYellow) {
      double rightSideOfYellow = yellowBuoyDetection.value().bbox_x1;
      if (!ifCloseToTheYellow) {
        ifCloseToTheYellow = true;
        courseBeforeYellowBuoy = state.currentHeading;
      }
      RCLCPP_INFO(nodePtr->get_logger(), "GOING AROUND THE YELLOW BUOY");
      if (rightSideOfYellow < 100) {
        turnID = TurnID::TURN_LEFT_SLOW;
      }
      else if (rightSideOfYellow >= 160 && rightSideOfYellow < 240) {
        turnID = TurnID::GO_STRAIGHT;
      }
      else if (rightSideOfYellow >= 160) {
        turnID = TurnID::TURN_RIGHT_SLOW;
      }
      ThrustHandlerForTurning(state, turnID);
     }
  }

  // means that we were close to the yellow but it ran out of our sight
  if (!yellowBuoyDetection.has_value() && ifCloseToTheYellow) {
    turnID = TurnID::TURN_LEFT_FAST;
    ThrustHandlerForTurning(state, turnID);
  }
}

void SpeedChallengeNavigator::SteerToGate(state::VesselState& state) {
  std::optional<l2_interfaces::msg::Detection> closestRed, closestGreen, blueBuoyDetection;
  double gateMid;
  // TODO: refactor once we differentiate between these two types of buoy
  std::vector<l2_interfaces::msg::Detection> redBuoyDetections;
  std::vector<l2_interfaces::msg::Detection> greenBuoyDetections;

  for (const auto& det : state.bigBuoyDetections) {
    if (det.detection_type == l2_interfaces::msg::Detection::DETECTION_TYPE_RED) {
      redBuoyDetections.push_back(det);
    }
    if (det.detection_type == l2_interfaces::msg::Detection::DETECTION_TYPE_GREEN) {
      greenBuoyDetections.push_back(det);
    }
    if (det.detection_type == l2_interfaces::msg::Detection::DETECTION_TYPE_BLUE) {
      blueBuoyDetection = det;
    }
  }
  RCLCPP_INFO(nodePtr->get_logger(),
              "redBuoyDetections: %ld, greenBuoyDetections: %ld, blueBuoy: %s",
              redBuoyDetections.size(), greenBuoyDetections.size(),
              blueBuoyDetection.has_value() ? "true" : "false");

  if (!redBuoyDetections.empty()) {
    closestRed = *std::max_element(
        std::begin(redBuoyDetections), std::end(redBuoyDetections),
        [](const auto& first, const auto& second) {
          return Area(first.bbox_x1 - first.bbox_x0, first.bbox_y1 - first.bbox_y0) <
                 Area(second.bbox_x1 - second.bbox_x0, second.bbox_y1 - second.bbox_y0);
        });
  }

  if (!greenBuoyDetections.empty()) {
    closestGreen = *std::max_element(
        std::begin(greenBuoyDetections), std::end(greenBuoyDetections),
        [](const auto& first, const auto& second) {
          return Area(first.bbox_x1 - first.bbox_x0, first.bbox_y1 - first.bbox_y0) <
                 Area(second.bbox_x1 - second.bbox_x0, second.bbox_y1 - second.bbox_y0);
        });
  }

  if (closestRed.has_value() && closestGreen.has_value()) {
    double redBboxMid = DetectionMidpoint(closestRed.value());
    double greenBboxMid = DetectionMidpoint(closestGreen.value());
    gateMid = MiddlePointOfTheGate(redBboxMid, greenBboxMid);
    // if (closestRed.value().distance < THRESHOLD_DISTANCE ||
    //     closestGreen.value().distance < THRESHOLD_DISTANCE) {
    // changed for sim
    if (closestRed.value().area > THRESHOLD_AREA || closestGreen.value().area > THRESHOLD_AREA) {
      RCLCPP_INFO(nodePtr->get_logger(), "CLOSE TO THE GATE");
      if (!gateCrossedStart) {
        gateCrossedStart = true;
      }
      else {
        ifCloseToTheGate = true;
      }
    }
  }
  else if (closestRed.has_value() && !closestGreen.has_value()) {
    double redBboxMid = DetectionMidpoint(closestRed.value());
    double symmetricalCoordsGreen;
    if (buoyManouvered) {
      symmetricalCoordsGreen = (redBboxMid > 320.0) ? 640.0 - redBboxMid : 640.0;
    }
    else {
      symmetricalCoordsGreen = (redBboxMid > 320.0) ? 640.0 : 640.0 - redBboxMid;
    }
    gateMid = MiddlePointOfTheGate(redBboxMid, symmetricalCoordsGreen);
    // if (closestRed.value().distance < THRESHOLD_DISTANCE) {
    // changed for sim
    if (closestRed.value().area > THRESHOLD_AREA) {
      if (!gateCrossedStart) {
        gateCrossedStart = true;
      }
      else {
        ifCloseToTheGate = true;
      }
    }
  }
  else if (!closestRed.has_value() && closestGreen.has_value()) {
    double greenBboxMid = DetectionMidpoint(closestGreen.value());
    double symmetricalCoordsRed;
    if (buoyManouvered) {
      symmetricalCoordsRed = (greenBboxMid < 320.0) ? 640.0 - greenBboxMid : 0.0;
    }
    else {
      symmetricalCoordsRed = (greenBboxMid < 320.0) ? 0.0 : 640.0 - greenBboxMid;
    }

    gateMid = MiddlePointOfTheGate(symmetricalCoordsRed, greenBboxMid);
    // if (closestGreen.value().distance < THRESHOLD_DISTANCE) {
    // changed for sim
    if (closestGreen.value().area > THRESHOLD_AREA) {
      if (!gateCrossedStart) {
        gateCrossedStart = true;
      }
      else {
        ifCloseToTheGate = true;
      }
    }
  }
  else if (!closestRed.has_value() && !closestGreen.has_value() && ifCloseToTheGate && buoyManouvered){
    ThrustHandler(state, SPEED, 0.0);
    if (endCounter >= 75){ //this node spins approximately 0.2s so it should be around 5s
      gateCrossedEnd = true;
    }
    endCounter++;
  }
  SteerToMidpoint(gateMid, state);
}

}  // namespace task
}  // namespace algo
