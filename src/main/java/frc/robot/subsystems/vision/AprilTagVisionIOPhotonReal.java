package frc.robot.subsystems.vision;

import static frc.robot.Constants.FieldMap.APRIL_TAG_LAYOUT_FIELD;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionMap.VisionConstants;
import frc.robot.util.AprilTagVisionHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.Optional;
import java.util.stream.IntStream;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// hopefully we don't actually need to change too much from sim :P
public class AprilTagVisionIOPhotonReal implements AprilTagVisionIO {

  private final PhotonCamera[] cameras;
  private final PhotonPoseEstimator[] poseEstimators;
  private double previousTimestamp = 0.0;

  private final VisionConstants[] constantsList;

  public AprilTagVisionIOPhotonReal(VisionConstants... constantsList) {
    this.constantsList = constantsList;

    cameras = new PhotonCamera[constantsList.length];

    poseEstimators = new PhotonPoseEstimator[cameras.length];

    for (int i = 0; i < cameras.length; i++) {
      var constants = constantsList[i];

      var camera = new PhotonCamera(constants.cameraName());
      cameras[i] = camera;

      poseEstimators[i] =
          new PhotonPoseEstimator(
              APRIL_TAG_LAYOUT_FIELD,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              camera,
              constants.robotToCamera());
      poseEstimators[i].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
  }

  @Override
  public void updateInputs(AprilTagIOInputsLogged inputs) {
    ArrayList<PoseEstimate> poseEstimates = new ArrayList<>();
    var poseEstimateOptionals = getPoses();

    for (int i = 0; i < cameras.length; i++) {
      var camera = cameras[i];

      PhotonPipelineResult result = camera.getLatestResult();
      double timestamp = result.getTimestampSeconds();

      // why check if enabled?
      // don't we want estimation to continue updating even while disabled?
      if (!result.targets.isEmpty() && DriverStation.isEnabled()) {
        if (poseEstimateOptionals[i].isEmpty()) continue;
        var estimatedPose = poseEstimateOptionals[i].get().estimatedPose;

        double averageTagDistance = 0.0;

        for (PhotonTrackedTarget target : result.targets) {
          var tagPose = APRIL_TAG_LAYOUT_FIELD.getTagPose(target.getFiducialId());
          if (tagPose.isEmpty()) continue;

          averageTagDistance +=
              estimatedPose
                  .getTranslation()
                  .toTranslation2d()
                  .getDistance(tagPose.get().toPose2d().getTranslation());
        }

        // invalid tags are still added to the average – how do we stop this?
        // ideally it doesn't matter in simulation since the field is preloaded
        // with the right tags, but like... :/
        averageTagDistance = averageTagDistance / result.targets.size();

        poseEstimates.add(
            new PoseEstimate(estimatedPose, timestamp, averageTagDistance, result.targets.size()));
      }
    }

    inputs.poseEstimates = poseEstimates;
  }

  public Optional<EstimatedRobotPose>[] getPoses() {
    var optionalPoseEstimates = new Optional[cameras.length];

    for (int i = 0; i < cameras.length; i++) {
      var estimatedRobotPose = poseEstimators[i].update();
      double latestTimestamp = cameras[i].getLatestResult().getTimestampSeconds();
      boolean isNewResult = Math.abs(latestTimestamp - previousTimestamp) > 1e-5;

      if (estimatedRobotPose.isPresent()) {
        getDebugField()
            .getObject("VisionEstimation")
            .setPose(estimatedRobotPose.get().estimatedPose.toPose2d());
      } else {
        if (isNewResult) getDebugField().getObject("VisionEstimation").setPoses();
      }

      if (isNewResult) previousTimestamp = latestTimestamp;

      optionalPoseEstimates[i] = estimatedRobotPose;
    }

    return optionalPoseEstimates;
  }

  public Field2d getDebugField() {
    // stale check the field across the cameras
    if (!SmartDashboard.getEntry("Real Field").exists()) {
      Field2d field = new Field2d();
      SmartDashboard.putData("Real Field", field);
      return field;
    }
    return (Field2d) SmartDashboard.getData("Real Field");
  }

  @Override
  public double[] getNoisyDistances() {
    // good luck freshies <3
    return IntStream.range(0, cameras.length)
        .mapToDouble(i -> constantsList[i].cameraType().getNoisyDistance())
        .toArray();
  }
}
