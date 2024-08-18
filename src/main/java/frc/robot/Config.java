package frc.robot;

import static frc.robot.GlobalConstants.ROBOT;

import frc.robot.OI.*;

public final class Config {

  public static final class Subsystems {

    public static final boolean DRIVETRAIN_ENABLED = true;
    public static final boolean VISION_ENABLED = true;
    public static final boolean GAME_PIECE_VISION_ENABLED = true;
    public static final boolean INTAKE_ENABLED = false;
    public static final boolean PIVOT_ENABLED = true;
    public static final boolean SHOOTER_ENABLED = true;
    public static final boolean CLIMBER_ENABLED = false;
    public static final boolean LEDS_ENABLED = true;

    public static final boolean PROTOTYPES_ENABLED = false;
  }

  public static final class Controllers {

    public static final boolean DRIVER_ENALBED = true;
    public static final int DRIVER_PORT = 0;

    public static final boolean OPERATOR_ENABLED = true;
    public static final int OPERATOR_PORT = 0;
    public static final boolean JOYSTICK_OPERATOR_ENABLED = false && OPERATOR_ENABLED;
    public static final boolean BOARD_OPERATOR_ENABLED =
        !JOYSTICK_OPERATOR_ENABLED && OPERATOR_ENABLED;

    public static DriverMap getDriverController() {
      return new SimControllerMap(DRIVER_PORT);
    }

    public static OperatorMap getOperatorController() {
      // return BOARD_OPERATOR_ENABLED
      //   ? new CommandBoardControllerSubsystem(OPERATOR_PORT)
      //   : new CommandXboxControllerSubsystem(OPERATOR_PORT);
      return switch (ROBOT) {
        case COMPBOT -> new BoardOperatorMap(OPERATOR_PORT);
        case DEVBOT -> new BoardOperatorMap(OPERATOR_PORT);
        case SIMBOT -> new SimControllerMap(OPERATOR_PORT);
      };
    }
  }
}
