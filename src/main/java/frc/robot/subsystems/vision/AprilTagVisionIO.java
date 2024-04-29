package frc.robot.subsystems.vision;

import static frc.robot.util.AprilTagVisionHelpers.toArray;
import static frc.robot.util.LimelightHelpers.toPose3D;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.util.AprilTagVisionHelpers.PoseEstimate;
import java.util.ArrayList;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface AprilTagVisionIO {
  class AprilTagIOInputsLogged implements LoggableInputs {

    ArrayList<PoseEstimate> poseEstimates = new ArrayList<>();

    @Override
    public void toLog(LogTable table) {
      table.put("NumEstimates", poseEstimates.size());
      for (int i = 0; i < poseEstimates.size(); i++) {
        PoseEstimate poseEstimate = poseEstimates.get(i);
        table.put("EstimatedPose/" + i, toArray(poseEstimate.pose()));
        table.put("Timestamp/" + i, poseEstimate.timestampSeconds());
        table.put("NumTags/" + i, poseEstimate.tagCount());
        table.put("AvgTagDistance/" + i, poseEstimate.averageTagDistance());
        // table.put("", AprilTagVisionHelper.PhotonLogging.visionConstantsToLog();
      }
    }

    @Override
    public void fromLog(LogTable table) {
      int estimatedPoseCount = table.get("NumEstimates", 0);
      for (int i = 0; i < estimatedPoseCount; i++) {
        Pose3d pose = toPose3D(table.get("EstimatedPose/" + i, new double[] {}));
        double timestamp = table.get("Timestamp/" + i, 0.0);
        int tagCount = table.get("NumTags/" + i, 0);
        double averageTagDistance = table.get("AvgTagDistance/" + i, 0.0);
        poseEstimates.add(new PoseEstimate(pose, timestamp, averageTagDistance, tagCount));
      }
    }
  }

  default void updateInputs(AprilTagIOInputsLogged inputs) {}

  Field2d getDebugField();

  double[] getNoisyDistances();
}
