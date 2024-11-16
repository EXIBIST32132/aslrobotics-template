package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  // let's only log the controller inputs for the leader
  @AutoLog
  public static class PivotIOInputs {
    public boolean leaderConnected = true;
    public boolean followerConnected = true;

    public double leaderPositionRad = 0.0;
    public double leaderTargetPositionRad = 0.0;
    public double encoderPositionRad = 0.0;

    public double leaderVelocityRadPerSec = 0.0;
    public double leaderAppliedVolts = 0.0;
    public double leaderCurrentAmps = 0.0;

    public double followerPositionRad = 0.0;
    public double followerVelocityRadPerSec = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(PivotIOInputs inputs);

  /** Run open loop at the specified voltage. */
  public void setVoltage(double volts);

  /** Run closed loop at the specified velocity. */
  public void setPosition(double angleRad, double ffVolts);

  /** Stop in open loop. */
  public default void stop() {}

  /** Set position PID constants. */
  public void configurePID(double kP, double kI, double kD);
}
