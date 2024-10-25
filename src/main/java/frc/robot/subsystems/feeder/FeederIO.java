package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public double feederAppliedVolts = 0.0;
    public double feederCurrentAmps = 0.0;
    public double feederVelocityRadPerSecond = 0.0;
  }

  public default void updateInputs(final FeederIOInputs inputs){}

  /** Run the feeder at a specified voltage */
  public default void setVoltage(final double volts){}

  public default void setVelocity(double radiansPerSecond, double ffVolts){}

  public default void configurePID(double kP, double kI, double kD) {}
}
