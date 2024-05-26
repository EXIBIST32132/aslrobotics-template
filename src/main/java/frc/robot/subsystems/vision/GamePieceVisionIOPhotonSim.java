package frc.robot.subsystems.vision;

import static frc.robot.Constants.FieldMap.Coordinates.SPEAKER;
import static frc.robot.Constants.VisionMap.GamePieceVisionMap.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.VisionMap.VisionConstants;
import frc.robot.Robot;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class GamePieceVisionIOPhotonSim implements GamePieceVisionIO {

  private final VisionSystemSim sim;
  private final PhotonCamera camera;
  private final PhotonCameraSim simCamera;
  public final Supplier<Pose2d> drivePoseSupplier;
  private double previousTimestamp = 0.0;

  private static final String NAME = "GamePieceVisionSim";

  private final VisionConstants constants;

  public GamePieceVisionIOPhotonSim(Supplier<Pose2d> drivePoseSupplier, VisionConstants constants) {
    this.constants = constants;
    sim = new VisionSystemSim(NAME);
    sim.addVisionTargets(
        new VisionTargetSim(
            new Pose3d(
                new Translation3d(
                    SPEAKER.getPose().getTranslation().getX(),
                    SPEAKER.getPose().getTranslation().getY(),
                    GAME_PIECE_HEIGHT),
                //                    FIELD_LENGTH_METERS / 2 /*+ Math.random() * 2 - 1*/,
                //                    FIELD_WIDTH_METERS / 2 /*+ Math.random() * 2 - 1*/,
                //                    GAME_PIECE_HEIGHT),
                new Rotation3d()),
            new TargetModel(GAME_PIECE_HEIGHT * 2)));

    camera = new PhotonCamera(constants.cameraName());

    SimCameraProperties cameraProperties = new SimCameraProperties();
    // SimCameraProperties.PERFECT_90DEG();
    cameraProperties.setCalibration(1080, 960, Rotation2d.fromDegrees(120));
    cameraProperties.setCalibError(0.5, 0.3);
    cameraProperties.setFPS(35.0);
    cameraProperties.setAvgLatencyMs(20.0);
    cameraProperties.setLatencyStdDevMs(5.0);

    simCamera = new PhotonCameraSim(camera, cameraProperties);

    // pv is *CLOCKWISE-POSITIVE*, which took me 3 entire hours to figure out.
    sim.addCamera(
        simCamera,
        new Transform3d(
            constants.robotToCamera().getTranslation(),
            constants.robotToCamera().getRotation().unaryMinus()));
    simCamera.enableDrawWireframe(false);

    this.drivePoseSupplier = drivePoseSupplier;
  }

  @Override
  public void updateInputs(GamePieceVisionIO.GamePieceVisionIOInputs inputs) {
    sim.update(drivePoseSupplier.get());
    PhotonPipelineResult result = camera.getLatestResult();
    double latestTimestamp = result.getTimestampSeconds();

    boolean isNewResult = Math.abs(latestTimestamp - previousTimestamp) > 1e-5;

    if (result.hasTargets()) {
      inputs.hasTarget = true;
      inputs.targetYaw = result.getBestTarget().getYaw();
      inputs.targetPitch = result.getBestTarget().getPitch();
    } else {
      inputs.hasTarget = false;
      inputs.targetPitch = 0;
      inputs.targetYaw = 0;
    }
  }

  @Override
  public VisionConstants getConstants() {
    return constants;
  }

  @Override
  public Field2d getDebugField() {
    return !Robot.isSimulation() ? null : sim.getDebugField();
  }
}
