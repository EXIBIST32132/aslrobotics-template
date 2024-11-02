package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double climberLeftPositionMeters = 0.0;
    public double climberRightPositionMeters = 0.0;
    public double climberLeftAppliedVolts = 0.0;
    public double climberRightAppliedVolts = 0.0;
    public double[] climberCurrentAmps =
        new double[] {}; // Log motors individually, useful for failure analysis
    public double climberSetpointPosition = 0.0;

    public boolean openLoopStatus = true;
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(ClimberIOInputs inputs);

  /** Sets the target of the climber * */
  public void setPosition(double climberPositionRad);

  /** Run open loop at the specified voltage. */
  public void setVoltage(double volts);

  public void setLeftVoltage(double volts);

  public void setRightVoltage(double volts);

  public void setHoming(boolean homingBool);

  public void resetEncoder(final double position);

  public default void resetEncoder() {
    resetEncoder(0);
  }

  public default boolean isCurrentLimited() {
    return false;
  }

  public default boolean isLeftCurrentLimited() {
    return false;
  }

  public default boolean isRightCurrentLimited() {
    return false;
  }

  public void configurePID(double kP, double kI, double kD);
}
