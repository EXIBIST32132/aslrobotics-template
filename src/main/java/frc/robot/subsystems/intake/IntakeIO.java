package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double intakeAppliedVolts = 0.0;
    public double intakeCurrentAmps = 0.0;
    public double intakeVelocityRadPerSecond = 0.0;
  }

  public void updateInputs(final IntakeIOInputs inputs);
  /** Run the intake at a specified voltage */
  public void setVoltage(final double volts);

  public void setVelocity(double radiansPerSecond, double ffVolts);

  public boolean hasNote();

  public void configurePID(double kP, double kI, double kD);
}
