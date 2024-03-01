#ifndef VESSEL_CONTROLLER_SPEED_CHALLENGE_NAVIGATOR_HPP_
#define VESSEL_CONTROLLER_SPEED_CHALLENGE_NAVIGATOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include "algo/algorithm.hpp"
#include "l2_interfaces/msg/detection.hpp"
#include "state/state_machine.hpp"

namespace algo {
namespace task {

enum class SpeedChallengeStateID {
  ENTERED_TASK,
  STEER_TO_GATE,
  GO_TO_THE_BUOY,
  MANOUVER_AROUND_BUOY,
  STEER_TO_GATE_RETURN,
  FINISHED_TASK
};

enum class TurnID {
  GO_STRAIGHT,
  TURN_RIGHT_SLOW,
  TURN_RIGHT_FAST,
  TURN_LEFT_SLOW,
  TURN_LEFT_FAST
};

constexpr double THRESHOLD_DISTANCE = 2.0;
constexpr double THRESHOLD_AREA = 500.0;
constexpr int DESIRED_NUMBER_OF_COUNTS = 10;

class SpeedChallengeNavigator : public Algorithm {
  std::shared_ptr<rclcpp::Node> nodePtr;
  SpeedChallengeStateID internalStateID;
  TurnID turnID;
  bool ifCloseToTheYellow;
  bool ifCloseToTheGate;
  bool gateCrossedStart;
  bool gateCrossedEnd;
  bool buoyManouvered;
  bool gateFound;
  bool goingRoundBlue;
  // counters for detection errors
  int counterForBlue = 0;
  int counterForYellow = 0;
  int endCounter = 0; //counter when we dont see the ending gate but we must pass through it
  rclcpp::Time smallBuoyTimestamp;

  using TransitionType = state::Transition<SpeedChallengeStateID, state::VesselState>;
  const state::StateMachine<SpeedChallengeStateID, state::VesselState> stateMachine{
      {TransitionType(SpeedChallengeStateID::ENTERED_TASK, SpeedChallengeStateID::STEER_TO_GATE,
                      [&](const auto& /*state*/) -> bool { return gateFound; }),
       TransitionType(SpeedChallengeStateID::STEER_TO_GATE, SpeedChallengeStateID::GO_TO_THE_BUOY,
                      [&](const auto& state) -> bool { return gateCrossedStart; }),
       TransitionType(SpeedChallengeStateID::GO_TO_THE_BUOY,
                      SpeedChallengeStateID::MANOUVER_AROUND_BUOY,
                      [&](const auto& state) -> bool { return ifCloseToTheYellow; }),
       TransitionType(SpeedChallengeStateID::MANOUVER_AROUND_BUOY,
                      SpeedChallengeStateID::STEER_TO_GATE_RETURN,
                      [&](const auto& state) -> bool { return buoyManouvered; }),
       TransitionType(SpeedChallengeStateID::STEER_TO_GATE_RETURN,
                      SpeedChallengeStateID::FINISHED_TASK,
                      [&](const auto& state) -> bool { return gateCrossedEnd; })}};

public:
  SpeedChallengeNavigator(rclcpp::Node* parentNode);
  void Run(state::VesselState& newState) override;

private:
  double courseBeforeYellowBuoy;
  void ThrustHandlerForTurning(state::VesselState& state, TurnID turnID);
  double DetectionMidpoint(const l2_interfaces::msg::Detection& detection);
  double MiddlePointOfTheGate(double midPointRed, double midPointGreen);
  void SteerToMidpoint(double midpoint, state::VesselState& state);
  void SteerToGate(state::VesselState& state);
  void SteerAndManouverToYellowBuoy(state::VesselState& state);
  void StationKeep(state::VesselState& state);
  bool FindGate(state::VesselState& state);
  void ThrustHandler(state::VesselState& state, double speed, double angle);
};

}  // namespace task
}  // namespace algo

#endif  // VESSEL_CONTROLLER_SPEED_CHALLENGE_NAVIGATOR_HPP_
