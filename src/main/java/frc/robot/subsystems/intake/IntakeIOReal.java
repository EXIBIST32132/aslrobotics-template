package frc.robot.subsystems.intake;

import com.revrobotics.*;

public class IntakeIOReal implements IntakeIO {
  private final CANSparkMax intake =
      new CANSparkMax(IntakeMap.INTAKE_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder Encoder = intake.getEncoder();
  private final SparkPIDController pid = intake.getPIDController();

  // private final DigitalInput beamBreak = new DigitalInput(0);

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // TODO: Make Inputs
  }

  @Override
  public void setVoltage(double volts) {
    intake.setVoltage(volts);
  }

  @Override
  public boolean hasNote() {
    // return beamBreak.get();
    return false;
  }
}
