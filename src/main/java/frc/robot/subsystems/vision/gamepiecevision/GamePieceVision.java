package frc.robot.subsystems.vision.gamepiecevision;

import static frc.robot.subsystems.vision.gamepiecevision.GamePieceVisionMap.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

public class GamePieceVision extends SubsystemBase {

  private final GamePieceVisionIO io;
  private final GamePieceVisionIOInputsAutoLogged inputs = new GamePieceVisionIOInputsAutoLogged();

  public GamePieceVision(GamePieceVisionIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.recordOutput("Vision/Target Yaw", inputs.targetYaw);
  }

  public double getNoteDistance() {
    return -PhotonUtils.calculateDistanceToTargetMeters(
        DETECTOR_LIMELIGHT_HEIGHT,
        GAME_PIECE_HEIGHT,
        DETECTOR_LIMELIGHT_PITCH,
        Units.degreesToRadians(-inputs.targetPitch));
  }

  public Transform2d getTargetToRobotOffset(Supplier<Pose2d> robotPose) {
    if (inputs.hasTarget) {
      // no helpers?
      double targetDistanceMeters = getNoteDistance();
      if (targetDistanceMeters > io.getConstants().cameraType().getNoisyDistance()) {
        io.getDebugField().getObject("NotePosition").setPoses();
        return new Transform2d();
      }
      Translation2d camToTargTrans =
          PhotonUtils.estimateCameraToTargetTranslation(
              targetDistanceMeters, Rotation2d.fromDegrees(inputs.targetYaw));
      Transform2d robotToNoteTransform =
          ROBOT_CENTER_TO_DETECTOR_LIMELIGHT_2D.plus(
              new Transform2d(camToTargTrans, Rotation2d.fromDegrees(0.0)));
      Rotation2d targetAngleRobotRelative =
          robotToNoteTransform.getTranslation().getAngle().plus(Rotation2d.fromDegrees(180));
      var noteRobotRelativePose =
          new Transform2d(robotToNoteTransform.getTranslation(), targetAngleRobotRelative);
      var ret =
          new Transform2d(
                  noteRobotRelativePose.getTranslation(),
                  robotPose.get().getRotation().minus(Rotation2d.fromDegrees(90)))
              .plus(ROBOT_CENTER_TO_DETECTOR_LIMELIGHT_2D.times(2));
      io.getDebugField()
          .getObject("NotePosition")
          .setPose(new Pose2d(ret.getTranslation(), ret.getRotation()));
      return new Transform2d(
          robotToNoteTransform.getTranslation(),
          robotToNoteTransform.getRotation().minus(targetAngleRobotRelative));
    }
    io.getDebugField().getObject("NotePosition").setPoses();
    return new Transform2d();
  }

  public boolean hasNote() {
    return inputs.hasTarget;
  }

  public double getNoteAngle() {
    return inputs.targetYaw;
  }
}
