package frc.robot.subsystems.feeder;

import com.revrobotics.*;

public class FeederIOReal implements FeederIO {
  private final CANSparkMax feeder =
      new CANSparkMax(FeederMap.FEEDER_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder Encoder = feeder.getEncoder();
  private final SparkPIDController pid = feeder.getPIDController();

  @Override
  public void updateInputs(FeederIO.FeederIOInputs inputs) {
    // TODO: Make Inputs
  }

  @Override
  public void setVoltage(double volts) {
    feeder.setVoltage(volts);
  }

  @Override
  public void setVelocity(double radiansPerSecond, double ffVolts) {
    pid.setReference(
        radiansPerSecond,
        CANSparkBase.ControlType.kVelocity,
        0,
        ffVolts,
        SparkPIDController.ArbFFUnits.kVoltage);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
  }
}
