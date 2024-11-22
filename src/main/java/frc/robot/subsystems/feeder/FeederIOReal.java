package frc.robot.subsystems.feeder;

import com.revrobotics.*;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;

public class FeederIOReal implements FeederIO {
  private final CANSparkMax feeder =
      new CANSparkMax(FeederMap.FEEDER_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final SparkPIDController pid = feeder.getPIDController();
  // private final DigitalInput beamBrake = new DigitalInput(1);

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
        Units.radiansPerSecondToRotationsPerMinute(radiansPerSecond),
        CANSparkBase.ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  public boolean hasNote() {
    // return beamBrake.get();
    return false;
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
  }

  @Override
  public void stop() {
    feeder.setVoltage(0.0);
  }
}
