package frc.robot.util;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import java.util.Arrays;
import java.util.Objects;

/**
 * This class provides utility methods and record classes for vision-related operations,
 * specifically for pose estimation using April Tags.
 */
public class AprilTagVisionHelpers {

  /**
   * Represents a pose estimate with additional information.
   *
   * @param pose The pose (position and orientation) estimate.
   * @param timestampSeconds The timestamp in seconds when the pose estimate was recorded.
   * @param averageTagDistance The average distance to the detected tags.
   * @param tagCount The IDs of the detected tags.
   */
  public record PoseEstimate(
      Pose3d pose, double timestampSeconds, double averageTagDistance, int tagCount) {
    /**
     * Checks if this pose estimate is equal to another object.
     *
     * @param obj The object to compare.
     * @return True if the objects are equal, false otherwise.
     */
    @Override
    public boolean equals(Object obj) {
      if (this == obj) {
        return true;
      }
      if (obj == null || getClass() != obj.getClass()) {
        return false;
      }
      PoseEstimate other = (PoseEstimate) obj;
      return (Double.compare(tagCount, other.tagCount) == 0
          && Objects.equals(pose, other.pose)
          && Double.compare(timestampSeconds, other.timestampSeconds) == 0
          && Double.compare(averageTagDistance, other.averageTagDistance) == 0);
    }

    /**
     * Computes the hash code of this PoseEstimate.
     *
     * @return The hash code value.
     */
    @Override
    public int hashCode() {
      return Objects.hash(
          Arrays.hashCode(toArray(pose)), timestampSeconds, averageTagDistance, tagCount);
    }
  }

  /**
   * Converts a Pose3d object to an array of doubles.
   *
   * @param pose The Pose3d object to convert.
   * @return The array of doubles representing the pose.
   */
  public static double[] toArray(Pose3d pose) {
    double[] result = new double[6];
    result[0] = pose.getTranslation().getX();
    result[1] = pose.getTranslation().getY();
    result[2] = pose.getTranslation().getZ();
    result[3] = Units.radiansToDegrees(pose.getRotation().getX());
    result[4] = Units.radiansToDegrees(pose.getRotation().getY());
    result[5] = Units.radiansToDegrees(pose.getRotation().getZ());
    return result;
  }

  public static Pose2d getPose3dToPose2d(Pose3d pose) {
    return new Pose2d(
        pose.getTranslation().getX(),
        pose.getTranslation().getY(),
        pose.getRotation().toRotation2d());
  }

  /**
   * Represents a timestamped vision update with pose and standard deviations.
   *
   * @param timestamp The timestamp of the vision update.
   * @param poseEstimate The pose estimate.
   * @param stdDevs The standard deviations matrix.
   */
  public record TimestampedVisionUpdate(
      double timestamp, Pose2d poseEstimate, Matrix<N3, N1> stdDevs) {
    /**
     * Returns a string representation of this vision update.
     *
     * @return The string representation.
     */
    @Override
    public String toString() {
      return ("VisionUpdate{"
          + "timestamp="
          + Double.toString(timestamp)
          + ", pose="
          + poseEstimate.toString()
          + ", stdDevs="
          + stdDevs.toString()
          + '}');
    }
  }

  /**
   * Cool idea by 8033, 5026, and the PoseEstimator docs – dynamically update the vision stddevs
   * based on the distance. In this case, we're scaling the deviations exponentially with distance,
   * so faraway tags are trusted <i>way</i> less than close ones.
   */
  public static record UnitDeviationParams(
      double distanceMultiplier, double eulerMultiplier, double minimum) {
    private double computeUnitDeviation(double averageDistance) {
      return Math.max(minimum, eulerMultiplier * Math.exp(averageDistance * distanceMultiplier));
    }
  }

  public static record MovingDeviationParams(
      double velocityMultiplier, double eulerMultiplier, double minimum) {
    private double computeVelocityDeviation(ChassisSpeeds robotVelocities) {
      return Math.max(
          minimum,
          eulerMultiplier
              * Math.exp(
                  velocityMultiplier
                      * Math.hypot(
                          robotVelocities.vxMetersPerSecond, robotVelocities.vyMetersPerSecond)));
    }
  }

  /**
   * @param xParams the dynamic deviation parameters for translation in the x-direction
   * @param yParams the dynamic deviation parameters for translation in the y-direction
   * @param thetaParams the dynamic deviation parameters for rotation
   */
  public static record TagCountDeviation(
      UnitDeviationParams xParams, UnitDeviationParams yParams, UnitDeviationParams thetaParams) {
    /**
     * @see TagCountDeviation
     * @param xyParams the dynamic deviation parameters for translation
     * @param thetaParams the dynamic deviation parameters for rotation
     */
    public TagCountDeviation(UnitDeviationParams xyParams, UnitDeviationParams thetaParams) {
      this(xyParams, xyParams, thetaParams);
    }

    public Matrix<N3, N1> computeDeviation(double averageDistance) {
      return MatBuilder.fill(
          Nat.N3(),
          Nat.N1(),
          xParams.computeUnitDeviation(averageDistance),
          yParams.computeUnitDeviation(averageDistance),
          thetaParams.computeUnitDeviation(averageDistance));
    }
  }

  /**
   * <b>NOT USED (YET(?))</b> <br>
   * <br>
   * A global log sender-receiver helper for PV because AKit doesn't support non-primitives or lists
   * of them as entries yet :(
   */
  public static class PhotonLogging {
    //    public static void photonTrackedTargetToLog(PhotonTrackedTarget target, LogTable table,
    // String name) {
    //      transform3dToLog(target.getBestCameraToTarget(), table, name);
    //      transform3dToLog(target.getAlternateCameraToTarget(), table, "Alt " + name);
    //      tagCornersToLog(target, table, name);
    //
    //      table.put("Tags/Yaw " + name, target.getYaw());
    //      table.put("Tags/Pitch " + name, target.getPitch());
    //      table.put("Tags/Area " + name, target.getArea());
    //      table.put("Tags/Skew " + name, target.getSkew());
    //      table.put("Tags/Fiducial ID " + name, target.getFiducialId());
    //      table.put("Tags/Pose Ambiguity " + name, target.getPoseAmbiguity());
    //    }
    //
    //    public static PhotonTrackedTarget photonTrackedTargetFromLog(LogTable table, String name)
    //    {
    //      double[] translation = table.get("Tags/Translation " + name, new double[3]);
    //      double[] rotation = table.get("Tags/Rotation " + name, new double[4]);
    //
    //      // erm... what the flip do we need this for?
    //      double[] altTranslation = table.get("Tags/Translation Alt " + name, new double[3]);
    //      double[] altRotation = table.get("Tags/Rotation Alt " + name, new double[4]);
    //
    //      List<TargetCorner> detectedCorners = new ArrayList<>();
    //      double[] detectedCornersX = table.get("Tags/Detected Corners X " + name, new double[4]);
    //      double[] detectedCornersY = table.get("Tags/Detected Corners Y " + name, new double[4]);
    //
    //      List<TargetCorner> minAreaRectCorners = new ArrayList<>();
    //      double[] minAreaRectCornersX =
    //        table.get("Tags/Min Area Rect Corners X " + name, new double[4]);
    //      double[] minAreaRectCornersY =
    //        table.get("Tags/Min Area Rect Corners Y " + name, new double[4]);
    //
    //      for (int i = 0; i < 4; i++) {
    //        detectedCorners.add(new TargetCorner(detectedCornersX[i], detectedCornersY[i]));
    //        minAreaRectCorners.add(new TargetCorner(minAreaRectCornersX[i],
    // minAreaRectCornersY[i]));
    //      }
    //
    //      Transform3d pose = transform3dFromArrays(translation, rotation);
    //      Transform3d altPose = transform3dFromArrays(altTranslation, altRotation);
    //      return new PhotonTrackedTarget(
    //        table.get("Tags/Yaw " + name, -1),
    //        table.get("Tags/Pitch " + name, -1),
    //        table.get("Tags/Area " + name, -1),
    //        table.get("Tags/Skew " + name, -1),
    //        table.get("Tags/Fiducial ID " + name, -1),
    //        pose,
    //        altPose,
    //        table.get("Tags/Pose Ambiguity " + name, -1),
    //        minAreaRectCorners,
    //        detectedCorners);
    //    }
    //
    //    public static void tagCornersToLog(PhotonTrackedTarget target, LogTable table, String
    // name) {
    //      double[] detectedCornersX = new double[4];
    //      double[] detectedCornersY = new double[4];
    //      double[] minAreaRectCornersX = new double[4];
    //      double[] minAreaRectCornersY = new double[4];
    //
    //      for (int i = 0; i < 4; i++) {
    //        detectedCornersX[i] = target.getDetectedCorners().get(i).x;
    //        detectedCornersY[i] = target.getDetectedCorners().get(i).y;
    //        minAreaRectCornersX[i] = target.getMinAreaRectCorners().get(i).x;
    //        minAreaRectCornersY[i] = target.getMinAreaRectCorners().get(i).y;
    //      }
    //
    //      table.put("Tags/Detected Corners X " + name, detectedCornersX);
    //      table.put("Tags/Detected Corners Y " + name, detectedCornersY);
    //      table.put("Tags/Min Area Rect Corners X " + name, minAreaRectCornersX);
    //      table.put("Tags/Min Area Rect Corners Y " + name, minAreaRectCornersY);
    //    }
    //
    //    // please come up with a better return type
    //    public static double[][] tagCornersFromLog(LogTable table, String name) {
    //      double[] detectedCornersX;
    //      double[] detectedCornersY;
    //      double[] minAreaRectCornersX;
    //      double[] minAreaRectCornersY;
    //
    //      detectedCornersX = table.get("Tags/Detected Corners X " + name, new double[] {});
    //      detectedCornersY = table.get("Tags/Detected Corners Y " + name, new double[] {});
    //      minAreaRectCornersX = table.get("Tags/Min Area Rect Corners X " + name, new double[]{});
    //      minAreaRectCornersY = table.get("Tags/Min Area Rect Corners Y " + name, new double[]{});
    //
    //      return new double[][] {
    //        detectedCornersX, detectedCornersY, minAreaRectCornersX, minAreaRectCornersY
    //      };
    //    }
    //
    //    public static void transform3dToLog(Transform3d transform3d, LogTable table, String name)
    //    {
    //      double[] rotation = new double[4];
    //      rotation[0] = transform3d.getRotation().getQuaternion().getW();
    //      rotation[1] = transform3d.getRotation().getQuaternion().getX();
    //      rotation[2] = transform3d.getRotation().getQuaternion().getY();
    //      rotation[3] = transform3d.getRotation().getQuaternion().getZ();
    //      table.put("Tags/Quaternion Rot " + name, rotation);
    //
    //      double[] translation = new double[3];
    //      translation[0] = transform3d.getTranslation().getX();
    //      translation[1] = transform3d.getTranslation().getY();
    //      translation[2] = transform3d.getTranslation().getZ();
    //      table.put("Tags/Translation " + name, translation);
    //    }
    //
    //    public static Transform3d transform3dFromArrays(double[] translation, double[] rotation) {
    //      return new Transform3d(
    //        new Translation3d(translation[0], translation[1], translation[2]),
    //        new Rotation3d(new Quaternion(rotation[0], rotation[1], rotation[2], rotation[3])));
    //    }
    //
    //    public static Transform3d transform3dFromLog(LogTable table, String name) {
    //      double[] rotation = table.get("Tags/Rotation " + name, new double[4]);
    //      double[] translation = table.get("Tags/Translation " + name, new double[3]);
    //      return transform3dFromArrays(translation, rotation);
    //    }
    //
    //    public static void visionConstantsToLog(VisionConstants constants, LogTable table) {
    //      table.put("Vision Constants/Name ", constants.cameraName());
    //      table.put("Vision Constants/Transform ", constants.robotToCamera());
    //      table.put("Vision Constants/Cam Type Index ", constants.cameraType().ordinal());
    //    }
    //
    //    public static VisionConstants visionConstantsFromLog(LogTable table) {
    //      return new VisionConstants(
    //        table.get("Vision Constants/Name ", "Default"),
    //        table.get("Vision Constants/Transform ", new Transform3d()),
    //        CameraType.getFromIndex(table.get("Vision Constants/Cam Type Index ", -1)));
    //    }
  }
}
