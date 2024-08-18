package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class PivotIOInputsAutoLogged extends PivotIO.PivotIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("PivotPositionRad", pivotPositionRad);
    table.put("PivotVelocityRadPerSec", pivotVelocityRadPerSec);
    table.put("PivotAppliedVolts", pivotAppliedVolts);
    table.put("PivotCurrentAmps", pivotCurrentAmps);
  }

  @Override
  public void fromLog(LogTable table) {
    pivotPositionRad = table.get("PivotPositionRad", pivotPositionRad);
    pivotVelocityRadPerSec = table.get("PivotVelocityRadPerSec", pivotVelocityRadPerSec);
    pivotAppliedVolts = table.get("PivotAppliedVolts", pivotAppliedVolts);
    pivotCurrentAmps = table.get("PivotCurrentAmps", pivotCurrentAmps);
  }

  public PivotIOInputsAutoLogged clone() {
    PivotIOInputsAutoLogged copy = new PivotIOInputsAutoLogged();
    copy.pivotPositionRad = this.pivotPositionRad;
    copy.pivotVelocityRadPerSec = this.pivotVelocityRadPerSec;
    copy.pivotAppliedVolts = this.pivotAppliedVolts;
    copy.pivotCurrentAmps = this.pivotCurrentAmps.clone();
    return copy;
  }
}
