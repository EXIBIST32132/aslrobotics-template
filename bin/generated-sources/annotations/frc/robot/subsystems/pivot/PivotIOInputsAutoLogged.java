package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class PivotIOInputsAutoLogged extends PivotIO.PivotIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {}

  @Override
  public void fromLog(LogTable table) {}

  public PivotIOInputsAutoLogged clone() {
    PivotIOInputsAutoLogged copy = new PivotIOInputsAutoLogged();
    return copy;
  }
}
