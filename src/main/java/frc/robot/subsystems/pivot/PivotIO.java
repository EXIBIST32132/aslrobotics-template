package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public double pivotPositionRad = 0.0;
    public double pivotVelocityRadPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double[] pivotCurrentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(
      PivotIOInputsAutoLogged
          inputs) {} // TODO: figure out which to use as in PivotIOSim it is this instead of
  // PivotIOInputs

  public default void updateInputs(PivotIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run closed loop at the specified velocity. */
  public default void setPosition(double angleRad, double ffVolts) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Set position PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}
