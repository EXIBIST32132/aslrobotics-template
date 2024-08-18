package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class PivotSubsystem extends SubsystemBase {

  private final PivotIO io;
  private final PivotVisualizer targetVisualizer, actualVisualizer;

  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private ArmFeedforward ffModel;

  public PivotSubsystem(PivotIO io) {
    this.io = io;
    ffModel = new ArmFeedforward(0, 0.83, 0.5);
    io.configurePID(5, 0, 0.125);

    actualVisualizer = new PivotVisualizer("ActualVisualizer");
    targetVisualizer = new PivotVisualizer("TargetVisualizer");
  }

  public double getPosition() {
    return inputs.pivotPositionRad;
  }

  public void setPosition(DoubleSupplier angleRad) {
    // Log flywheel setpoint
    Logger.recordOutput("Pivot/Setpoint", angleRad.getAsDouble());
    targetVisualizer.update(angleRad.getAsDouble());
    io.setPosition(angleRad.getAsDouble(), ffModel.calculate(angleRad.getAsDouble(), 0));
  }

  public void stop() {
    io.stop();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);

    Logger.recordOutput("Pivot/position", inputs.pivotPositionRad);
    actualVisualizer.update(inputs.pivotPositionRad);
  }
}
