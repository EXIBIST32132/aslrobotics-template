package frc.robot.subsystems.feeder;

import com.revrobotics.*;
import edu.wpi.first.math.filter.Debouncer;

public class FeederIOReal implements FeederIO {
  private final CANSparkMax feeder =
      new CANSparkMax(FeederMap.FEEDER_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder encoder = feeder.getEncoder();
  private final SparkPIDController pid = feeder.getPIDController();

  // Debouncer to filter out noise or temporary spikes in current
  private final Debouncer currentDebouncer =
          new Debouncer(0.2, Debouncer.DebounceType.kRising); // 200ms debounce

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

  public boolean hasNote(){
    return currentDebouncer.calculate(
            feeder.getOutputCurrent()
                    >50);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
  }
}
