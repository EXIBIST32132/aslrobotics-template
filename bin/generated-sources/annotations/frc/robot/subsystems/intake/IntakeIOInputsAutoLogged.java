package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeIOInputsAutoLogged extends IntakeIO.IntakeIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("IntakeAppliedVolts", intakeAppliedVolts);
    table.put("IntakeCurrentAmps", intakeCurrentAmps);
    table.put("IntakeVelocityRadPerSecond", intakeVelocityRadPerSecond);
  }

  @Override
  public void fromLog(LogTable table) {
    intakeAppliedVolts = table.get("IntakeAppliedVolts", intakeAppliedVolts);
    intakeCurrentAmps = table.get("IntakeCurrentAmps", intakeCurrentAmps);
    intakeVelocityRadPerSecond =
        table.get("IntakeVelocityRadPerSecond", intakeVelocityRadPerSecond);
  }

  public IntakeIOInputsAutoLogged clone() {
    IntakeIOInputsAutoLogged copy = new IntakeIOInputsAutoLogged();
    copy.intakeAppliedVolts = this.intakeAppliedVolts;
    copy.intakeCurrentAmps = this.intakeCurrentAmps;
    copy.intakeVelocityRadPerSecond = this.intakeVelocityRadPerSecond;
    return copy;
  }
}
