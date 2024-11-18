package frc.robot.subsystems.feeder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class FeederSubsystem extends SubsystemBase {

  private final FeederIO io;
  private final FeederIO.FeederIOInputs inputs = new FeederIO.FeederIOInputs();


  public FeederSubsystem(FeederIO io) {
    this.io = io;
    this.io.configurePID(FeederMap.kP, FeederMap.kP, FeederMap.kD);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // Add logging or telemetry if needed
  }

  // Stops Feeder
  public Command stop(){
    return Commands.runOnce(()->io.setVelocity(0,0));
  }

  // runs with velocity
  public Command runCMD(double vel) {
    return Commands.runOnce(() -> io.setVelocity(vel, 0));
  }
  /** Trigger based on current draw (beam brake alternative using current detection) */
  public Trigger hasNote() {
    return new Trigger(io::hasNote); // TODO: Find port of beam break and set hasNote return type to actually return when broken
  }
}
