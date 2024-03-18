package frc.robot.subsystems.intake;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeIOInputsAutoLogged extends IntakeIO.IntakeIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("IntakeVelocityRadPerSecond", intakeVelocityRadPerSecond);
    table.put("IntakeAppliedVolts", intakeAppliedVolts);
    table.put("IntakeCurrentAmps", intakeCurrentAmps);
  }

  @Override
  public void fromLog(LogTable table) {
    intakeVelocityRadPerSecond = table.get("IntakeVelocityRadPerSecond", intakeVelocityRadPerSecond);
    intakeAppliedVolts = table.get("IntakeAppliedVolts", intakeAppliedVolts);
    intakeCurrentAmps = table.get("IntakeCurrentAmps", intakeCurrentAmps);
  }

  public IntakeIOInputsAutoLogged clone() {
    IntakeIOInputsAutoLogged copy = new IntakeIOInputsAutoLogged();
    copy.intakeVelocityRadPerSecond = this.intakeVelocityRadPerSecond;
    copy.intakeAppliedVolts = this.intakeAppliedVolts;
    copy.intakeCurrentAmps = this.intakeCurrentAmps;
    return copy;
  }
}
