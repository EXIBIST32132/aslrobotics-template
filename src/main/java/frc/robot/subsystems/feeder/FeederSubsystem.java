package frc.robot.subsystems.feeder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeMode;
import org.littletonrobotics.junction.Logger;

public class FeederSubsystem extends SubsystemBase {

  public enum IntakeMode {
    OFF(0.0), // Intake is off
    FORWARD(12.0), // Maximum forward voltage
    REVERSE(-12.0); // Maximum reverse voltage

    final double voltage;

    IntakeMode(double voltage) {
      this.voltage = voltage;
    }
  }

  private final FeederIO io;
  private final String name;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
  private IntakeMode currentState = IntakeMode.OFF;

  // Debouncer to filter out noise or temporary spikes in current
  private final Debouncer currentDebouncer =
      new Debouncer(0.4, DebounceType.kFalling); // 200ms debounce

  public FeederSubsystem(String name, FeederIO io) {
    this.name = name;
    this.io = io;
  }

  /** Trigger based on current draw (beam brake alternative using current detection) */
  public Trigger hasNote() {
    return new Trigger(
        () -> io.hasNote() || currentDebouncer.calculate(inputs.feederCurrentAmps > 40));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);

    // Add logging or telemetry if needed
    io.setVoltage(currentState.voltage);
  }

  /** Set intake to a specified mode using the enum */
  private Command setFeederMode(IntakeMode mode) {
    return Commands.runOnce(() -> currentState = mode, this);
  }

  /** Stop the intake */
  public Command stop() {
    return setFeederMode(IntakeMode.OFF);
  }

  /** Run the intake in reverse */
  public Command reverse() {
    return setFeederMode(IntakeMode.REVERSE);
  }

  /** Run the intake at maximum speed */
  public Command forward() {
    return setFeederMode(IntakeMode.FORWARD);
  }
}
