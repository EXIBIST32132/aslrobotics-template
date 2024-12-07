package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  // let's only log the controller inputs for the leader
  @AutoLog
  public static class PivotIOInputs {
    public double positionRad = 0.0;
    public double absolutePositionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(PivotIOInputs inputs);

  /** Run open loop at the specified voltage. */
  public void setVoltage(double volts);
}
