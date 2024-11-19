package frc.robot.subsystems.climber;

import com.revrobotics.*;

public class ClimberIOReal implements ClimberIO {
  private final CANSparkMax leftClimber =
      new CANSparkMax(ClimberMap.LEFT_CLIMBER, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax rightClimber =
      new CANSparkMax(ClimberMap.RIGHT_CLIMBER, CANSparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder lEncoder = leftClimber.getEncoder();
  private final RelativeEncoder rEncoder = rightClimber.getEncoder();
  private final SparkPIDController lPid = leftClimber.getPIDController();
  private final SparkPIDController rPid = rightClimber.getPIDController();
  private double climberPosition = 0.0;

  public ClimberIOReal() {
    leftClimber.restoreFactoryDefaults();
    rightClimber.restoreFactoryDefaults();

    leftClimber.setCANTimeout(250);
    rightClimber.setCANTimeout(250);

    leftClimber.setInverted(false);
    rightClimber.setInverted(false);

    leftClimber.enableVoltageCompensation(12.0);
    rightClimber.enableVoltageCompensation(12.0);

    leftClimber.setSmartCurrentLimit(30);
    rightClimber.setSmartCurrentLimit(30);

    leftClimber.burnFlash();
    rightClimber.burnFlash();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberLeftPositionMeters = lEncoder.getPosition();
    inputs.climberRightPositionMeters = rEncoder.getPosition();
    inputs.climberCurrentAmps[0] = leftClimber.getOutputCurrent();
    inputs.climberCurrentAmps[1] = rightClimber.getOutputCurrent();
    inputs.climberLeftAppliedVolts = leftClimber.getBusVoltage();
    inputs.climberRightAppliedVolts = rightClimber.getBusVoltage();

    inputs.climberSetpointPosition = climberPosition;
  }

  @Override
  public void setPosition(double climberPositionRad) {
    lPid.setReference(climberPositionRad, CANSparkBase.ControlType.kPosition, 0);
    rPid.setReference(climberPositionRad, CANSparkBase.ControlType.kPosition, 0);
    climberPosition = climberPositionRad;
  }

  @Override
  @Deprecated
  public void setVoltage(double volts) {
    leftClimber.setVoltage(volts);
    rightClimber.setVoltage(volts);
  }

  @Override
  @Deprecated
  public void setLeftVoltage(double volts) {
    leftClimber.setVoltage(volts);
  }

  @Override
  @Deprecated
  public void setRightVoltage(double volts) {
    rightClimber.setVoltage(volts);
  }

  @Override
  @Deprecated
  public void setHoming(boolean homingBool) {} // TODO: Figure out wtf is set Homing

  @Override
  public void resetEncoder(final double position) {
    lEncoder.setPosition(0);
    rEncoder.setPosition(0);
  }

  @Override
  public boolean isCurrentLimited() {
    return leftClimber.getOutputCurrent()
        > 50; // TODO: find a way to get current limits from a motor
  }

  @Override
  @Deprecated
  public boolean isLeftCurrentLimited() {
    return true;
  } // TODO: find a way to get current limits from a motor

  @Override
  @Deprecated
  public boolean isRightCurrentLimited() {
    return true;
  } // TODO: find a way to get current limits from a motor

  @Override
  public void configurePID(double kP, double kI, double kD) {
    // Left PID Values
    lPid.setP(kP, 0);
    lPid.setI(kI, 0);
    lPid.setD(kD, 0);
    lPid.setFF(0, 0);
    // Right PID Values
    rPid.setP(kP, 0);
    rPid.setI(kI, 0);
    rPid.setD(kD, 0);
    rPid.setFF(0, 0);
  }
}
