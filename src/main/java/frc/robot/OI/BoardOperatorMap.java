package frc.robot.OI;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class BoardOperatorMap extends CommandGenericHID implements OperatorMap {
  public BoardOperatorMap(int port) {
    super(port);
  }

  @Override
  public Trigger shoot() {
    return null;
  }

  @Override
  public Trigger pivotToSpeaker() {
    return null;
  }
}
