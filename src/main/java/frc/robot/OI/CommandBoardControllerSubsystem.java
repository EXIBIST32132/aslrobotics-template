package frc.robot.OI;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

/**
 * A wrapper to interface with board-based controllers, such as our operator console in the 2024-25
 * season.
 */
public class CommandBoardControllerSubsystem extends CommandGenericHID implements Subsystem {
  public CommandBoardControllerSubsystem(int port) {
    super(port);
  }
}
