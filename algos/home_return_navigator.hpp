#pragma once
#ifndef VESSEL_CONTROLLER_HOME_RETURN_NAVIGATOR_HPP_
#define VESSEL_CONTROLLER_HOME_RETURN_NAVIGATOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include "algo/algorithm.hpp"
#include "l2_interfaces/msg/detection.hpp"
#include "state/state_machine.hpp"

constexpr double PI = 3.141592653589793238462643383279502884197;
constexpr double THRESHOLD_COURSE = 10.0;
constexpr double THRESHOLD_DISTANCE = 2.0;



namespace algo {
namespace task {

enum class HomeReturnStateID { ENTERED_TASK, GOING_TO_HOME_WAYPOINT, STEER_TO_GATE, FINISHED_TASK };

class HomeReturnNavigator : public Algorithm {
  rclcpp::Node* nodePtr;
  HomeReturnStateID internalStateID;
  bool gateFound;
  bool atHome;
  bool gateCrossed;

  enum class TurnID {
    GO_STRAIGHT,
    TURN_RIGHT_SLOW,
    TURN_RIGHT_FAST,
    TURN_LEFT_SLOW,
    TURN_LEFT_FAST
  };

  using TransitionType = state::Transition<HomeReturnStateID, state::VesselState>;
  const state::StateMachine<HomeReturnStateID, state::VesselState> stateMachine{
      {TransitionType(HomeReturnStateID::ENTERED_TASK, HomeReturnStateID::GOING_TO_HOME_WAYPOINT,
                      [&](const auto& /*state*/) -> bool { return true; }),
       TransitionType(HomeReturnStateID::GOING_TO_HOME_WAYPOINT, HomeReturnStateID::STEER_TO_GATE,
                      [&](const auto& state) -> bool { return gateFound; }),
       TransitionType(HomeReturnStateID::STEER_TO_GATE, HomeReturnStateID::GOING_TO_HOME_WAYPOINT,
                      [&](const auto& state) -> bool { return !gateFound; }),
       TransitionType(HomeReturnStateID::STEER_TO_GATE, HomeReturnStateID::GOING_TO_HOME_WAYPOINT,
                      [&](const auto& state) -> bool { return gateCrossed; }),
       TransitionType(HomeReturnStateID::GOING_TO_HOME_WAYPOINT, HomeReturnStateID::FINISHED_TASK,
                      [&](const auto& state) -> bool { return atHome; })}};

public:
  HomeReturnNavigator(rclcpp::Node* parentNode);
  void Run(state::VesselState& newState) override;
  void NavigateHomeWaypoint(state::VesselState& state);
  void SearchForGate(state::VesselState& state);
  void SteerToGate(state::VesselState& state);
  void ThrustHandler(state::VesselState& state, double speed, double angle);
  double GetHeadingTo(double x0, double y0, double x1, double y1);
  double DistToHome(state::VesselState& state);
  double DetectionMidpoint(const l2_interfaces::msg::Detection& detection);
  double MiddlePointOfTheGate(double midPointLeft, double midPointRight);
};

}  // namespace task
}  // namespace algo

#endif  // VESSEL_CONTROLLER_HOME_RETURN_NAVIGATOR_HPP_
