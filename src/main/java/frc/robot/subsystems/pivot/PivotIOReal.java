package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotMap.Constants.ABSOLUTE_ENCODER_OFFSET;
import static frc.robot.subsystems.pivot.PivotMap.Hardware.*;
import static java.lang.Math.PI;

import com.revrobotics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class PivotIOReal implements PivotIO {
  private final CANSparkFlex
      leader = new CANSparkFlex(LEFT_PIVOT_ID, CANSparkLowLevel.MotorType.kBrushless),
      follower = new CANSparkFlex(RIGHT_PIVOT_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder encoder = leader.getEncoder();
  private final DutyCycleEncoder absEncoder = new DutyCycleEncoder(ENCODER_PORT);

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
    inputs.appliedVolts = leader.getBusVoltage();
    inputs.currentAmps = leader.getOutputCurrent();
    inputs.positionRad = leader.getEncoder().getPosition();
    inputs.absolutePositionRad = (absEncoder.get() + ABSOLUTE_ENCODER_OFFSET) * 2 * PI;
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  public double getEncoderPosition() {
    return absEncoder.getAbsolutePosition() + absEncoder.getPositionOffset();
  }
}
