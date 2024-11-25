package frc.robot.OI;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class BoardOperatorMap extends CommandGenericHID implements OperatorMap {
  public BoardOperatorMap(int port) {
    super(port);
  }

  @Override
  public Trigger shoot() {
    return button(5);
  }

  @Override
  public Trigger prepShot() {
    return button(7);
  }

  // TODO: add intake button
  @Override
  public Trigger intake() {
    return button(8);
  }
}
