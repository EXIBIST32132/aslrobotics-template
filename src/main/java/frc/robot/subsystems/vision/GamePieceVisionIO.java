package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

/** Keep game piece alignment ROBOT-RELATIVE! */
public interface GamePieceVisionIO {
  @AutoLog
  public static class GamePieceVisionIOInputs {

    public boolean hasTarget = false;
    public double targetYaw = 0.0;
    public double targetPitch = 0.0;
  }

  public default void updateInputs(GamePieceVisionIOInputs inputs) {}
}
