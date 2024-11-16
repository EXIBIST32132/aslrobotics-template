package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotMap.Hardware.*;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class PivotIOReal implements PivotIO {
  private final CANSparkFlex
      leader = new CANSparkFlex(LEFT_PIVOT_ID, CANSparkLowLevel.MotorType.kBrushless),
      follower = new CANSparkFlex(RIGHT_PIVOT_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder encoder = leader.getEncoder();
  private final DutyCycleEncoder absEncoder = new DutyCycleEncoder(ENCODER_PORT);
  private final SparkPIDController pid = leader.getPIDController();

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
  }

  @Override
  public void updateInputs(PivotIO.PivotIOInputs inputs) {
    inputs.leaderAppliedVolts = leader.getBusVoltage();
    inputs.leaderCurrentAmps = leader.getOutputCurrent();
    inputs.leaderPositionRad = absEncoder.getAbsolutePosition();
    inputs.leaderVelocityRadPerSec = encoder.getVelocity();
  }

  @Override
  public void setPosition(double climberPositionRad, double ffVolts) {
    pid.setReference(
        climberPositionRad,
        CANSparkBase.ControlType.kPosition,
        0,
        ffVolts,
        SparkPIDController.ArbFFUnits.kVoltage);
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  public void setHoming(boolean homingBool) {}

  @Override
  public void configurePID(double kP, double kI, double kD) {
    // Left PID Values
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0, 0);
  }

  @Override
  public void stop() {
    leader.setVoltage(0);
    follower.setVoltage(0);
  }
}
