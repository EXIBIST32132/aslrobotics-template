package frc.robot.subsystems.vision.gamepiecevision;

import static frc.robot.util.LimelightHelpers.*;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.VisionMap.CameraConstants;

public class GamePieceVisionIOLimelightReal implements GamePieceVisionIO {

  private final CameraConstants constants;

  public GamePieceVisionIOLimelightReal(CameraConstants constants) {
    this.constants = constants;
  }

  @Override
  public void updateInputs(GamePieceVisionIOInputs inputs) {
    if ((boolean) (inputs.hasTarget = getTV(constants.cameraName()))) {
      inputs.targetYaw = getTX(constants.cameraName());
      inputs.targetPitch = getTY(constants.cameraName());
    }
  }

  @Override
  public CameraConstants getConstants() {
    return constants;
  }

  @Override
  public Field2d getDebugField() {
    // stale check the field across the cameras
    if (!SmartDashboard.getEntry("Real Field").exists()) {
      Field2d field = new Field2d();
      SmartDashboard.putData("Real Field", field);
      return field;
    }
    return (Field2d) SmartDashboard.getData("Real Field");
  }
}
