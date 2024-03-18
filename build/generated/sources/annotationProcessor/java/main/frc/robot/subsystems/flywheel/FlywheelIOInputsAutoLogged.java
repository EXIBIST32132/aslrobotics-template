package frc.robot.subsystems.flywheel;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class FlywheelIOInputsAutoLogged extends FlywheelIO.FlywheelIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("ShooterPositionRad", shooterPositionRad);
    table.put("ShooterVelocityRadPerSec", shooterVelocityRadPerSec);
    table.put("ShooterAppliedVolts", shooterAppliedVolts);
    table.put("ShooterCurrentAmps", shooterCurrentAmps);
  }

  @Override
  public void fromLog(LogTable table) {
    shooterPositionRad = table.get("ShooterPositionRad", shooterPositionRad);
    shooterVelocityRadPerSec = table.get("ShooterVelocityRadPerSec", shooterVelocityRadPerSec);
    shooterAppliedVolts = table.get("ShooterAppliedVolts", shooterAppliedVolts);
    shooterCurrentAmps = table.get("ShooterCurrentAmps", shooterCurrentAmps);
  }

  public FlywheelIOInputsAutoLogged clone() {
    FlywheelIOInputsAutoLogged copy = new FlywheelIOInputsAutoLogged();
    copy.shooterPositionRad = this.shooterPositionRad;
    copy.shooterVelocityRadPerSec = this.shooterVelocityRadPerSec;
    copy.shooterAppliedVolts = this.shooterAppliedVolts;
    copy.shooterCurrentAmps = this.shooterCurrentAmps.clone();
    return copy;
  }
}
