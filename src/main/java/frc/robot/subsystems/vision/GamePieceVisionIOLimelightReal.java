package frc.robot.subsystems.vision;

import static frc.robot.util.LimelightHelpers.*;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  @Override
  public VisionConstants getConstants() {
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
