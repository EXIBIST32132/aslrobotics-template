package frc.robot.subsystems.vision;

import static frc.robot.Constants.FieldMap.APRIL_TAG_LAYOUT_FIELD;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionMap.VisionConstants;
import frc.robot.util.AprilTagVisionHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagVisionIOPhotonReal implements AprilTagVisionIO {

  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
  private double previousTimestamp = 0.0;

  private final VisionConstants constants;

  public AprilTagVisionIOPhotonReal(VisionConstants constants) {
    this.constants = constants;
    camera = new PhotonCamera(constants.cameraName());

    var camera = new PhotonCamera(constants.cameraName());

    poseEstimator =
        new PhotonPoseEstimator(
            APRIL_TAG_LAYOUT_FIELD,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            constants.robotToCamera());

    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void updateInputs(AprilTagIOInputsLogged inputs) {
    ArrayList<PoseEstimate> poseEstimates = new ArrayList<>();

    var poseEstimateOptionals = getPoses();

    PhotonPipelineResult result = camera.getLatestResult();
    double timestamp = result.getTimestampSeconds();

    if (!result.targets.isEmpty() && DriverStation.isEnabled()) {
      if (poseEstimateOptionals.isEmpty()) return;
      var poseEstimate = poseEstimateOptionals.get().estimatedPose;

      double averageTagDistance = 0.0;

      for (PhotonTrackedTarget target : result.targets) {
        var tagPose = APRIL_TAG_LAYOUT_FIELD.getTagPose(target.getFiducialId());
        if (tagPose.isEmpty()) continue;

        averageTagDistance +=
            poseEstimate
                .getTranslation()
                .toTranslation2d()
                .getDistance(tagPose.get().toPose2d().getTranslation());
      }

      averageTagDistance = averageTagDistance / result.targets.size();

      poseEstimates.add(
          new PoseEstimate(poseEstimate, timestamp, averageTagDistance, result.targets.size()));
    }

    inputs.poseEstimates = poseEstimates;
  }

  public Optional<EstimatedRobotPose> getPoses() {
    Optional<EstimatedRobotPose> optionalPoseEstimates;

    var poseEstimate = poseEstimator.update();
    double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - previousTimestamp) > 1e-5;

    if (poseEstimate.isPresent()) {
      getDebugField()
          .getObject("VisionEstimation")
          .setPose(poseEstimate.get().estimatedPose.toPose2d());
    } else {
      if (newResult) getDebugField().getObject("VisionEstimation").setPoses();
    }

    if (newResult) previousTimestamp = latestTimestamp;

    optionalPoseEstimates = poseEstimate;

    return optionalPoseEstimates;
  }

  public Field2d getDebugField() {
    Field2d field = new Field2d();
    SmartDashboard.putData("Field", field);
    return field;
  }

  @Override
  public String getName() {
    return constants.cameraName();
  }

  @Override
  public double[] getNoisyDistances() {
    return new double[] {constants.cameraType().getNoisyDistance()};
  }
}
