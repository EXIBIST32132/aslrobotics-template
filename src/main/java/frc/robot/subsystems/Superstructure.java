package frc.robot.subsystems;

import static frc.robot.subsystems.Superstructure.SuperStates.IDLING;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
  /**
   * For use during autonomous and the Superstructure's periodic. The action (non-idle) entries end
   * in "ing" intentionally – if the robot is not in the state of actively transitioning between
   * states, it's idling.
   */
  public static enum SuperStates {
    IDLING,
    INTAKING,
    SHOOTING
  }

  /**
   * For use during teleop to modify the current SuperState. Each one requests a state in {@link
   * SuperStates}. REQ_NONE is the absence of
   */
  public static enum StateRequests {
    REQ_NONE,
    REQ_IDLE,
    REQ_INTAKE,
    REQ_SHOOT
  }

  private SuperStates currentState = IDLING;

  @Override
  public void periodic() {
    switch (currentState) {
      case IDLING -> {}
      case INTAKING -> {}
      case SHOOTING -> {}
    }
  }
}
