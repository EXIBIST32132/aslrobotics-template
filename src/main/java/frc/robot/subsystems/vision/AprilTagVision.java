package frc.robot.subsystems.vision;

import static frc.robot.Constants.FieldMap.FIELD_LENGTH_METERS;
import static frc.robot.Constants.FieldMap.FIELD_WIDTH_METERS;
import static frc.robot.Constants.VisionMap.AprilTagVisionMap.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.AprilTagVisionIO.AprilTagIOInputsLogged;
import frc.robot.util.AprilTagVisionHelpers.PoseEstimate;
import frc.robot.util.AprilTagVisionHelpers.TimestampedVisionUpdate;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Handles the {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator} interface using either
 * of the AprilTagVisionIOs.
 */
public class AprilTagVision extends SubsystemBase {

  private static final String NAME = "AprilTagVision/Instance ";

  private final AprilTagVisionIO[] ios;
  private final AprilTagIOInputsLogged[] inputs;

  public AprilTagVision(AprilTagVisionIO... ios) {
    this.ios = ios;
    inputs = new AprilTagIOInputsLogged[ios.length];
  }

  @Override
  public void periodic() {
    /*
     5712 did some kind of time-bound processing here, only using
     inputs that were completed within a 0.1s deadline per periodic call.
     do we need to do this as well? would it actually help looptime,
     or just complicate stuff between akit's replaying and the fpga?
    */
    for (int i = 0; i < ios.length; i++) {
      ios[i].updateInputs(inputs[i]);
      Logger.processInputs(NAME + i, inputs[i]);
    }
  }

  // TODO multitag enforcement as well or allow lowest ambiguity?
  /**
   * Invalidates the PoseEstimate if we're reported to be outside the field boundaries, above or
   * under the field, or too far from the tag(s) to have an accurate reading.
   *
   * @param poseEstimate the estimate to be analyzed
   * @return whether the input estimate should be considered
   */
  public boolean isInvalidEstimate(PoseEstimate poseEstimate) {
    Pose3d pose = poseEstimate.pose();
    return (pose.getX() < 0
        || pose.getX() > FIELD_LENGTH_METERS
        || pose.getY() < 0
        || pose.getY() > FIELD_WIDTH_METERS
        || poseEstimate.averageTagDistance() > MAX_DISTANCE_CUTOFF
        || Math.abs(poseEstimate.pose().getZ()) > 0.25);
  }

  public List<TimestampedVisionUpdate> getProcessedPoseEstimates() {
    List<TimestampedVisionUpdate> visionUpdates = new ArrayList<>();
    for (int i = 0; i < ios.length; i++) {
      for (PoseEstimate poseEstimate : inputs[i].poseEstimates) {
        if (isInvalidEstimate(poseEstimate)
            || poseEstimate.averageTagDistance() < ios[i].getNoisyDistances()[i]) continue;
        double timestamp = poseEstimate.timestampSeconds();
        Pose3d robotPose = poseEstimate.pose();
        visionUpdates.add(
            new TimestampedVisionUpdate(
                timestamp, robotPose.toPose2d(), calculateVisionStdDevs(poseEstimate)));
      }
    }
    return visionUpdates;
  }

  public Matrix<N3, N1> calculateVisionStdDevs(PoseEstimate poseEstimate) {
    return TAG_COUNT_DEVIATION_PARAMS
        .get(MathUtil.clamp(poseEstimate.tagCount() - 1, 0, 2))
        .computeDeviation(poseEstimate.averageTagDistance());
  }
}
