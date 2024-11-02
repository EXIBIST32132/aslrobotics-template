// change between sim and sparkmax files

package frc.robot.subsystems.climber;

public class ClimberIOSim implements ClimberIO {
  @Override
  public void updateInputs(ClimberIOInputs inputs) {}

  @Override
  public void setPosition(double climberPositionRad) {}

  @Override
  public void setVoltage(double volts) {}

  @Override
  public void setLeftVoltage(double volts) {}

  @Override
  public void setRightVoltage(double volts) {}

  @Override
  public void setHoming(boolean homingBool) {}

  @Override
  public void resetEncoder(double position) {}

  @Override
  public void resetEncoder() {
    ClimberIO.super.resetEncoder();
  }

  @Override
  public boolean isCurrentLimited() {
    return ClimberIO.super.isCurrentLimited();
  }

  @Override
  public boolean isLeftCurrentLimited() {
    return ClimberIO.super.isLeftCurrentLimited();
  }

  @Override
  public boolean isRightCurrentLimited() {
    return ClimberIO.super.isRightCurrentLimited();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {}
}
