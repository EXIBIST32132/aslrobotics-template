package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.OI.CommandBoardControllerSubsystem;
import frc.robot.OI.CommandXboxControllerSubsystem;

public final class Config {
  public static final class Subsystems {
    public static final boolean DRIVETRAIN_ENABLED = false;
    public static final boolean VISION_ENABLED = false;
    public static final boolean INTAKE_ENABLED = false;
    public static final boolean SHOOTER_ENABLED = false;
    public static final boolean CLIMBER_ENABLED = false;
    public static final boolean LEDS_ENABLED = false;
  }

  public static final class Controllers {
    public static final boolean DRIVER_ENALBED = false;
    public static final int DRIVER_PORT = 0;

    public static final boolean OPERATOR_ENABLED = false;
    public static final int OPERATOR_PORT = 1;
    public static final boolean JOYSTICK_OPERATOR_ENABLED = false && OPERATOR_ENABLED;
    public static final boolean BOARD_OPERATOR_ENABLED =
        !JOYSTICK_OPERATOR_ENABLED && OPERATOR_ENABLED;

    public static CommandGenericHID getDriverController() {
      return new CommandXboxControllerSubsystem(DRIVER_PORT);
    }

    public static CommandGenericHID getOperatorController() {
      return BOARD_OPERATOR_ENABLED
          ? new CommandBoardControllerSubsystem(OPERATOR_PORT)
          : new CommandXboxControllerSubsystem(OPERATOR_PORT);
    }
  }
}
