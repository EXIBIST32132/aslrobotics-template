package frc.robot.subsystems.vision.gamepiecevision;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.vision.VisionMap.CameraConstants;
import org.littletonrobotics.junction.AutoLog;

/** Keep game piece alignment ROBOT-RELATIVE! */
public interface GamePieceVisionIO {
  @AutoLog
  public static class GamePieceVisionIOInputs {

    public boolean hasTarget = false;
    public double targetYaw = 0.0;
    public double targetPitch = 0.0;
  }

  public CameraConstants getConstants();

  public Field2d getDebugField();

  public default void updateInputs(GamePieceVisionIOInputs inputs) {}
}
