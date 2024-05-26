package frc.robot.subsystems.prototypes;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;

@Deprecated
// package-private because Prototypes.java should be the only usage
class SendableMotor implements Sendable, Subsystem {

  public enum Motor {
    NEO(new Constants(0, 41, -5676, 5676)),
    NEO550(new Constants(0, 41, -11000, 11000)),
    VORTEX(new Constants(0, 7167, -6784, 6784));

    //    BASE_FALCON(new Constants(0, 2047, -6380, 6380)),
    //    PRO_FALCON(new Constants(0, 2047, -6080, 6080)),
    //    BASE_KRAKEN(new Constants(0, 2047, -6000, 6000)),
    //    PRO_KRAKEN(new Constants(0, 2047, -5800, 5800));

    final Constants constants;

    Motor(Constants constants) {
      this.constants = constants;
    }

    record Constants(int lowerPos, int upperPos, int lowerVel, int upperVel) {}
  }

  private final CANSparkBase motor;
  private final SparkPIDController pidController;
  private final SlewRateLimiter m_percentSRL, m_velocitySRL;
  private final TrapezoidProfile.Constraints m_positionConstraints;

  private boolean enabled = true;

  public double goalPercent, goalPos, goalVel;
  public double actualPos, actualVel;
  public final double LOWER_POS_LIMIT, UPPER_POS_LIMIT, LOWER_VEL_LIMIT, UPPER_VEL_LIMIT;

  public SendableMotor(
      int id,
      Motor type,
      int lowerPosLimit,
      int upperPosLimit,
      int lowerVelLimit,
      int upperVelLimit) {
    motor =
        type == Motor.VORTEX
            ? new CANSparkFlex(id, MotorType.kBrushless)
            : new CANSparkMax(id, MotorType.kBrushless);
    pidController = motor.getPIDController();
    m_percentSRL = new SlewRateLimiter(0.25);
    m_velocitySRL = new SlewRateLimiter(0.25);
    m_positionConstraints = new TrapezoidProfile.Constraints(500.0, 1000.0);

    this.LOWER_POS_LIMIT = lowerPosLimit;
    this.UPPER_POS_LIMIT = upperPosLimit;
    this.LOWER_VEL_LIMIT = lowerVelLimit;
    this.UPPER_VEL_LIMIT = upperVelLimit;
  }

  public SendableMotor(int id, Motor type) {
    this(
        id,
        type,
        type.constants.lowerPos,
        type.constants.upperPos,
        type.constants.lowerVel,
        type.constants.upperVel);
  }

  public SendableMotor(int id) {
    this(id, Motor.VORTEX);
  }

  public Command profiledPositionCommand() {
    return new TrapezoidProfileCommand(
            new TrapezoidProfile(m_positionConstraints),
            state -> pidController.setReference(state.position, ControlType.kPosition),
            () -> new TrapezoidProfile.State(goalPos, 0.0),
            () -> new TrapezoidProfile.State(actualPos, actualVel),
            this)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
        .onlyWhile(() -> enabled);
  }

  public Command positionRangeCommand() {
    return profiledPositionCommand()
        .alongWith(
            Commands.repeatingSequence(
                Commands.runOnce(
                    () -> goalPos = goalPos == LOWER_POS_LIMIT ? UPPER_POS_LIMIT : LOWER_POS_LIMIT),
                Commands.waitSeconds(3.0)))
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
        .onlyWhile(() -> enabled);
  }

  public Command profiledPercentCommand() {
    return this.run(() -> motor.set(m_percentSRL.calculate(goalPercent)))
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
        .onlyWhile(() -> enabled);
  }

  public Command profiledVelocityCommand() {
    return this.run(
            () ->
                pidController.setReference(m_velocitySRL.calculate(goalVel), ControlType.kVelocity))
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
        .onlyWhile(() -> enabled);
  }

  public Command toggleEnable() {
    return Commands.runOnce(() -> enabled = !enabled);
  }

  @Override
  public void periodic() {
    actualPos = motor.getEncoder().getPosition();
    actualVel = motor.getEncoder().getVelocity();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("kP", pidController::getP, pidController::setP);
    builder.addDoubleProperty("kI", pidController::getI, pidController::setI);
    builder.addDoubleProperty("kD", pidController::getD, pidController::setD);
    builder.addDoubleProperty("kF", pidController::getFF, pidController::setFF);
  }
}
