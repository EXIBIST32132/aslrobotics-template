package frc.robot.subsystems.pivot;

import static frc.robot.GlobalConstants.ROBOT;

public final class PivotMap {

  public static final class Sim {}

  public static final class Hardware {

    public static final int LEFT_PIVOT_ID = 31;
    public static final int RIGHT_PIVOT_ID = 32;
    public static final int ENCODER_PORT = 1;
  }

  public static final class Constants {

    // TODO change these after tuning!
    public static final Gains Gains =
        switch (ROBOT) {
          case COMPBOT -> new Gains(0.0005, 0, 0.125, 0, 0.5, 0.83);
          case DEVBOT -> new Gains(0, 0, 0, 0, 0, 0);
          case SIMBOT -> new Gains(0, 0, 0, 0, 0, 0);
        };

    public record Gains(double kP, double kI, double kD, double kS, double kV, double kG) {}
  }
}
