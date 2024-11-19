package frc.robot.subsystems.intake;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOReal implements IntakeIO {
  private final CANSparkMax intake =
      new CANSparkMax(IntakeMap.INTAKE_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder Encoder = intake.getEncoder();
  private final SparkPIDController pid = intake.getPIDController();

  private final DigitalInput beamBreak = new DigitalInput(0);

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // TODO: Make Inputs
  }

  @Override
  public void setVoltage(double volts) {
    intake.setVoltage(volts);
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
  public boolean hasNote(){
    return beamBreak.get();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
  }
}
