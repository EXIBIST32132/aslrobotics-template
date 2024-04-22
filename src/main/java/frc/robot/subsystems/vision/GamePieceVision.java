package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionMap.GamePieceVisionMap.*;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;

public class GamePieceVision extends SubsystemBase {

  private final GamePieceVisionIO io;
  private final GamePieceVisionIOInputsAutoLogged inputs = new GamePieceVisionIOInputsAutoLogged();

  public GamePieceVision(GamePieceVisionIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
  }

  public Transform2d getTargetToRobotOffset() {
    return inputs.hasTarget
        ? LimelightHelpers.cameraToTargetTranslationOffset(
                LimelightHelpers.targetToCameraDistance(
                    DETECTOR_LIMELIGHT_HEIGHT, DETECTOR_LIMELIGHT_PITCH, 0, inputs.targetPitch),
                inputs.targetYaw)
            .plus(
                new Transform2d(
                    DETECTOR_LIMELIGHT_TO_ROBOT_CENTER.getTranslation(),
                    DETECTOR_LIMELIGHT_TO_ROBOT_CENTER.getRotation()))
        : new Transform2d();
  }
}
