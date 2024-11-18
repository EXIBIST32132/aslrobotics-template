package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  public enum ClimberMode {
    ZERO(0.0), // Climber is off
    MAXHEIGHT(1.0); // Maximum Height
    final double height;

    ClimberMode(double height) {
      this.height = height;
    }
  }

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
    io.configurePID(ClimberMap.kP, ClimberMap.kI, ClimberMap.kD); // TODO: change to correct PID Values
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // Add logging if necessary
  }

  /** Set climber to a specified mode using the enum */
  private void setClimberMode(ClimberMode mode) {
    // Set intake velocity based on the selected mode
    io.setPosition(mode.height);
  }

  /** Set the position of the climber to Zero */
  public void stop() {
    setClimberMode(ClimberMode.ZERO);
  }

  /** Set the position of the climber to its max height */
  public void MaxHeight() {
    setClimberMode(ClimberMode.MAXHEIGHT);
  }

  /** resets encoders */
  public void resetEncoder() {
    io.resetEncoder();
  }

  /** sets the motors to mirror one another */
  public void setHoming(boolean homingBool) {
    io.setHoming(homingBool);
  }
}
