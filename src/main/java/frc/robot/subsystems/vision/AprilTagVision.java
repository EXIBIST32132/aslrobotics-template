package frc.robot.subsystems.vision;

import static frc.robot.Constants.FieldMap.FIELD_LENGTH_METERS;
import static frc.robot.Constants.FieldMap.FIELD_WIDTH_METERS;
import static frc.robot.Constants.VisionMap.AprilTagVisionMap.TAG_COUNT_DEVIATION_PARAMS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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

  private static final String LogKey = "AprilTagVision";

  private final AprilTagVisionIO io;
  private final AprilTagIOInputsLogged inputs = new AprilTagIOInputsLogged();

  public AprilTagVision(AprilTagVisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    /*
     * note that 5712 did some kind of time-bound processing here, only using
     * inputs that were completed within a 0.1s deadline per periodic call.
     * note as well that they used multiple ios in the same vision system,
     * instead of a vararg of constants. would it be worth the re-refactoring?
     */
    io.updateInputs(inputs);
    Logger.processInputs(LogKey, inputs);
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
        || Math.abs(poseEstimate.pose().getZ()) > 0.25);
  }

  public List<TimestampedVisionUpdate> processPoseEstimates() {
    List<TimestampedVisionUpdate> visionUpdates = new ArrayList<>();

    // TODO DANGER: arraylist being processed against standard arrays - check ordering!
    var poseEstimates = inputs.poseEstimates;

    for (int i = 0; i < inputs.poseEstimates.size(); i++) {
      if (isInvalidEstimate(poseEstimates.get(i))
          || poseEstimates.get(i).averageTagDistance() > io.getNoisyDistances()[i]) continue;

      double timestamp = poseEstimates.get(i).timestampSeconds();
      Pose3d robotPose = poseEstimates.get(i).pose();

      Logger.recordOutput(
          LogKey + "/Deviations/x/" + i, calculateVisionStdDevs(poseEstimates.get(i)).get(0, 0));
      Logger.recordOutput(
          LogKey + "/Deviations/y/" + i, calculateVisionStdDevs(poseEstimates.get(i)).get(1, 0));
      Logger.recordOutput(
          LogKey + "/Deviations/theta/" + i,
          calculateVisionStdDevs(poseEstimates.get(i)).get(2, 0));
      Logger.recordOutput(LogKey + "/IsValid/" + i, !isInvalidEstimate(poseEstimates.get(i)));

      visionUpdates.add(
          new TimestampedVisionUpdate(
              timestamp, robotPose.toPose2d(), calculateVisionStdDevs(poseEstimates.get(i))));
    }
    return visionUpdates;
  }

  public Field2d getDebugField() {
    return io.getDebugField();
  }

  public Matrix<N3, N1> calculateVisionStdDevs(PoseEstimate poseEstimate) {
    return TAG_COUNT_DEVIATION_PARAMS
        .get(MathUtil.clamp(poseEstimate.tagCount() - 1, 0, 2))
        .computeDeviation(poseEstimate.averageTagDistance());
  }
}
