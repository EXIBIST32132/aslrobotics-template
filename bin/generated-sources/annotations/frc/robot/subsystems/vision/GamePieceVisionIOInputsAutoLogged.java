package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GamePieceVisionIOInputsAutoLogged extends GamePieceVisionIO.GamePieceVisionIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("HasTarget", hasTarget);
    table.put("TargetYaw", targetYaw);
    table.put("TargetPitch", targetPitch);
  }

  @Override
  public void fromLog(LogTable table) {
    hasTarget = table.get("HasTarget", hasTarget);
    targetYaw = table.get("TargetYaw", targetYaw);
    targetPitch = table.get("TargetPitch", targetPitch);
  }

  public GamePieceVisionIOInputsAutoLogged clone() {
    GamePieceVisionIOInputsAutoLogged copy = new GamePieceVisionIOInputsAutoLogged();
    copy.hasTarget = this.hasTarget;
    copy.targetYaw = this.targetYaw;
    copy.targetPitch = this.targetPitch;
    return copy;
  }
}
