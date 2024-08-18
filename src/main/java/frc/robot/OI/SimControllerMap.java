package frc.robot.OI;

import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble;
import static edu.wpi.first.wpilibj2.command.Commands.startEnd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class SimControllerMap extends CommandXboxController implements DriverMap, OperatorMap {

  public SimControllerMap(int port) {
    super(port);
  }

  @Override
  public Trigger alignToGamePiece() {
    return button(7);
  }

  @Override
  public Trigger alignToSpeaker() {
    return button(8);
  }

  @Override
  public DoubleSupplier getXAxis() {
    return () -> -getLeftY();
  }

  @Override
  public DoubleSupplier getYAxis() {
    return () -> -getLeftX();
  }

  @Override
  public DoubleSupplier getRotAxis() {
    return () -> -getRawAxis(2);
  }

  @Override
  public Trigger pathToAmp() {
    return button(14);
  }

  @Override
  public Trigger pathToSource() {
    return null;
  }

  @Override
  public Trigger pathToSpeaker() {
    return null;
  }

  @Override
  public Trigger pivotToSpeaker() {
    return button(5);
  }

  @Override
  public Trigger shoot() {
    return button(1);
  }

  @Override
  public Trigger stopWithX() {
    return button(4);
  }

  @Override
  public Trigger testButton() {
    return button(1);
  }

  // would we need to mutex this through a subsys req if we switch to the maple swerve skeleton?
  @Override
  public Command rumble() {
    return startEnd(
        () -> getHID().setRumble(kBothRumble, 1), () -> getHID().setRumble(kBothRumble, 0));
  }
}
