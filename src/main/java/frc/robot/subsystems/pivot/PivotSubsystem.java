package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotMap.Constants.Gains;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class PivotSubsystem extends SubsystemBase {

  public static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Pivot/FeedbackGains/kP/", Gains.kP());
  public static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Pivot/FeedbackGains/kI/", Gains.kI());
  public static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Pivot/FeedbackGains/kD/", Gains.kD());
  public static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Pivot/FeedforwardGains/kS/", Gains.kS());
  public static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Pivot/FeedforwardGains/kV/", Gains.kV());
  public static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Pivot/FeedforwardGains/kG/", Gains.kG());
  private final PivotIO io;
  private final PivotVisualizer actualVisualizer, targetVisualizer, setpointVisualizer;

  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private ArmFeedforward ffModel;

  private double goalAngle;

  public PivotSubsystem(PivotIO io) {
    this.io = io;
    ffModel = new ArmFeedforward(kS.get(), kG.get(), kV.get());
    io.configurePID(kP.get(), kI.get(), kD.get());

    actualVisualizer = new PivotVisualizer("ActualVisualizer");
    targetVisualizer = new PivotVisualizer("TargetVisualizer");
    setpointVisualizer = new PivotVisualizer("SetpointVisualizer");
  }

  public double getPosition() {
    return inputs.leaderPositionRad;
  }

  public void setPosition(DoubleSupplier angleRad) {
    // Log flywheel setpoint
    inputs.leaderTargetPositionRad = angleRad.getAsDouble();
    io.setPosition(angleRad.getAsDouble(), ffModel.calculate(angleRad.getAsDouble(), 0));
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> io.setVoltage(voltage));
  }

  public void stop() {
    io.stop();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);

    Logger.recordOutput("Pivot/position", inputs.leaderPositionRad);
    Logger.recordOutput("Pivot/setpoint", inputs.leaderTargetPositionRad);

    actualVisualizer.update(inputs.leaderPositionRad);
    targetVisualizer.update(inputs.leaderTargetPositionRad);
    setpointVisualizer.update(goalAngle);
  }
}
