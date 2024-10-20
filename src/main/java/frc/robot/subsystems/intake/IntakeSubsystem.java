package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

public class IntakeSubsystem extends SubsystemBase {

  public enum IntakeMode {
    OFF(0.0),       // Intake is off
    FAST(12.0),     // Maximum forward voltage
    REVERSE(-12.0); // Maximum reverse voltage

    final double voltage;

    IntakeMode(double voltage) {
      this.voltage = voltage;
    }
  }

  private final IntakeIO io;
  private final IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();
  private IntakeMode currentState = IntakeMode.OFF;

  // Debouncer to filter out noise or temporary spikes in current
  private final Debouncer currentDebouncer = new Debouncer(0.2, DebounceType.kRising); // 200ms debounce

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // Add logging or telemetry if needed
  }

  /** Set intake to a specified mode using the enum */
  public void setIntakeMode(IntakeMode mode) {
    currentState = mode;
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

  /** Trigger based on current draw (beam brake alternative using current detection) */
  public Trigger hasNote() {
    return new Trigger(() ->
            this.currentState == IntakeMode.FAST  // Detect only while intake is running forward
                    && currentDebouncer.calculate(inputs.intakeCurrentAmps > 40)  // Debounced current detection
    );
  }
}
