package frc.robot.subsystems.feeder;

import com.revrobotics.*;

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

  public boolean hasNote() {
    // return beamBrake.get();
    return false;
  }
}
