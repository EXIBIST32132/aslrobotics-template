package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  public enum IntakeMode {
    OFF(0.0),           // Intake is off
    FAST(12),         // Maximum forward velocity (radians per second)
    REVERSE(-12);     // Maximum reverse velocity (negative radians per second)

    final double voltage;

    IntakeMode(double voltage) {
      this.voltage = voltage;
    }
  }

  private final IntakeIO io;
  private final IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // Add logging if necessary
  }

  /** Set intake to a specified mode using the enum */
  public void setIntakeMode(IntakeMode mode) {
    // Set intake velocity based on the selected mode
    io.setVoltage(mode.voltage);
  }

  /** Stop the intake */
  public void stop() {
    setIntakeMode(IntakeMode.OFF);
  }

  /** Run the intake in reverse */
  public void reverse() {
    setIntakeMode(IntakeMode.REVERSE);
  }

  /** Run the intake at maximum speed */
  public void fast() {
    setIntakeMode(IntakeMode.FAST);
  }
}
