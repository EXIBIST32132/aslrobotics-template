package frc.robot.subsystems.vision.apriltagvision;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.vision.VisionMap;
import frc.robot.util.AprilTagVisionHelpers;
import java.util.List;

// TODO tune all of these!!
public final class AprilTagVisionMap {

  public static final boolean LEFT_CAM_ENABLED = true;
  public static final VisionMap.CameraConstants LEFT_CAM_CONSTANTS =
      new VisionMap.CameraConstants(
          "lefttagcam",
          new Transform3d(
              new Translation3d(0.306, -0.3, 0.15),
              new Rotation3d(0, degreesToRadians(-28), degreesToRadians(-30))),
          VisionMap.CameraType.TELEPHOTO_9281);

  public static final boolean RIGHT_CAM_ENABLED = true;
  public static final VisionMap.CameraConstants RIGHT_CAM_CONSTANTS =
      new VisionMap.CameraConstants(
          "righttagcam",
          new Transform3d(
              new Translation3d(0.306, 0.3, 0.15),
              new Rotation3d(0, degreesToRadians(-28), degreesToRadians(30))),
          VisionMap.CameraType.TELEPHOTO_9281);

  public static final List<AprilTagVisionHelpers.TagCountDeviation> TAG_COUNT_DEVIATION_PARAMS =
      List.of(
          // 1 tag
          new AprilTagVisionHelpers.TagCountDeviation(
              new AprilTagVisionHelpers.UnitDeviationParams(0.35, 0.40, 0.90),
              new AprilTagVisionHelpers.UnitDeviationParams(0.50, 0.70, 1.50)),
          // 2 tags
          new AprilTagVisionHelpers.TagCountDeviation(
              new AprilTagVisionHelpers.UnitDeviationParams(0.35, 0.1, 0.4),
              new AprilTagVisionHelpers.UnitDeviationParams(0.50, 0.70, 1.50)),
          // 3+ tags
          new AprilTagVisionHelpers.TagCountDeviation(
              new AprilTagVisionHelpers.UnitDeviationParams(0.25, 0.07, 0.25),
              new AprilTagVisionHelpers.UnitDeviationParams(0.15, 1.0, 1.50)));

  //      public static final UnitDeviationParams MOVING_DEVIATION_PARAMS =
  //          new UnitDeviationParams(
  //              MOVING_DEVIATION_VELOCITY_MULTIPLIER, MOVING_DEVIATION_EULER_MULTIPLIER, 1);
  public static final double MOVING_DEVIATION_EULER_MULTIPLIER = 0.5;
  public static final double MOVING_DEVIATION_VELOCITY_MULTIPLIER = 0.5;
  public static final double TURNING_DEVIATION_EULER_MULTIPLIER = 0.5;
  public static final double TURNING_DEVIATION_VELOCITY_MULTIPLIER = 0.5;
  static final double MAX_AMBIGUITY_CUTOFF = 0.05;
}
