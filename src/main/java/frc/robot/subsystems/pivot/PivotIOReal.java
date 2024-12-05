package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotMap.Constants.ABSOLUTE_ENCODER_OFFSET;
import static frc.robot.subsystems.pivot.PivotMap.Hardware.*;
import static java.lang.Math.PI;

import com.revrobotics.*;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class PivotIOReal implements PivotIO {
  private final CANSparkFlex
      leader = new CANSparkFlex(LEFT_PIVOT_ID, CANSparkLowLevel.MotorType.kBrushless),
      follower = new CANSparkFlex(RIGHT_PIVOT_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder encoder = leader.getEncoder();
  private final DutyCycleEncoder absEncoder = new DutyCycleEncoder(ENCODER_PORT);
  private final PivotMotor pivot = new PivotMotor(this);

  public PivotIOReal() {
    leader.restoreFactoryDefaults();
    follower.restoreFactoryDefaults();

    leader.setCANTimeout(250);
    follower.setCANTimeout(250);

    leader.setInverted(false);
    follower.follow(leader, true);

    leader.setIdleMode(CANSparkMax.IdleMode.kBrake);
    follower.setIdleMode(CANSparkMax.IdleMode.kBrake);

    leader.enableVoltageCompensation(12.0);
    follower.enableVoltageCompensation(12.0);

    leader.setSmartCurrentLimit(60);
    follower.setSmartCurrentLimit(60);

    leader.burnFlash();
    follower.burnFlash();

    absEncoder.setPositionOffset(ABSOLUTE_ENCODER_OFFSET);
  }

  @Override
  public void updateInputs(PivotIO.PivotIOInputs inputs) {
    inputs.leaderAppliedVolts = leader.getBusVoltage();
    inputs.leaderCurrentAmps = leader.getOutputCurrent();
    inputs.leaderPositionRad = (absEncoder.get()) * 2 * PI;
    inputs.leaderVelocityRadPerSec = encoder.getVelocity();
  }

  @Override
  public void setPosition(double pivotPositionRad) {
    pivot.setPosition(pivotPositionRad);
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void stop() {
    leader.setVoltage(0);
    follower.setVoltage(0);
  }

  public double getEncoderPosition() {
    return absEncoder.getAbsolutePosition() + absEncoder.getPositionOffset();
  }
}

class PivotMotor extends ProfiledPIDSubsystem {
  private PivotIOReal io;
  private ArmFeedforward pivotFeedforward =
      new ArmFeedforward(0, PivotMap.Constants.Gains.kG(), PivotMap.Constants.Gains.kV(), 0);

  public PivotMotor(PivotIOReal io) {
    super(
        new ProfiledPIDController(
            PivotMap.Constants.Gains.kP(),
            PivotMap.Constants.Gains.kI(),
            PivotMap.Constants.Gains.kD(),
            PivotMap.Constants.TrapProf));
    // From last years code, how they made the pivot
    this.io = io;
    getController().setIZone(PivotMap.Constants.kIZone);
    getController()
        .setTolerance(PivotMap.Constants.POSITION_TOLERANCE, PivotMap.Constants.VELOCITY_TOLERANCE);

    setGoal(PivotMap.Constants.ABSOLUTE_ENCODER_OFFSET);
    enable();
  }

  @Override
  protected void useOutput(double v, TrapezoidProfile.State state) {
    var feedforward = pivotFeedforward.calculate(state.position, state.velocity);

    io.setVoltage(v + feedforward);
  }

  @Override
  protected double getMeasurement() {
    return io.getEncoderPosition();
  }

  public void setPosition(double setpoint) {
    setGoal(setpoint + ABSOLUTE_ENCODER_OFFSET);
  }
}
