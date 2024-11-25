package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public double feederAppliedVolts = 0.0;
    public double feederCurrentAmps = 0.0;
    public double feederVelocityRadPerSecond = 0.0;
  }

  public void updateInputs(final FeederIOInputs inputs);

  public void setVoltage(double volts);

  public boolean hasNote();
}
