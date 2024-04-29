package frc.robot.subsystems.vision;

import static frc.robot.util.LimelightHelpers.*;

import frc.robot.Constants.VisionMap.VisionConstants;

public class GamePieceVisionIOLimelightReal implements GamePieceVisionIO {

  private final VisionConstants constants;

  public GamePieceVisionIOLimelightReal(VisionConstants constants) {
    this.constants = constants;
  }

  @Override
  public void updateInputs(GamePieceVisionIOInputs inputs) {
    if ((boolean) (inputs.hasTarget = getTV(constants.cameraName()))) {
      inputs.targetYaw = getTX(constants.cameraName());
      inputs.targetPitch = getTY(constants.cameraName());
    }
  }
}
