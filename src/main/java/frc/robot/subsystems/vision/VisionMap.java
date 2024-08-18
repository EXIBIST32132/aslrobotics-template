package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

public final class VisionMap {

  public static enum CameraType {
    OV2311(5.0),
    OV9281(5.0),
    LIMELIGHT(2.0),
    LIMELIGHT_3G(3.0),
    TELEPHOTO_2311(5.0),
    TELEPHOTO_9281(5.0),
    TELEPHOTO_LIMELIGHT(5.0),
    TELEPHOTO_LIMELIGHT_3G(5.0),
    UNKNOWN(0.0);

    final double noisyDistance;

    CameraType(double noisyDistance) {
      this.noisyDistance = noisyDistance;
    }

    public double getNoisyDistance() {
      return noisyDistance;
    }
  }

  public record CameraConstants(
      String cameraName, Transform3d robotToCamera, CameraType cameraType) {}
}
