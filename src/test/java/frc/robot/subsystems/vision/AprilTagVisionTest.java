package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.VisionMap.CameraType;
import frc.robot.Constants.VisionMap.VisionConstants;
import frc.robot.util.AprilTagVisionHelpers.PoseEstimate;
import org.junit.jupiter.api.Test;

public class AprilTagVisionTest {
  AprilTagVision vision =
      new AprilTagVision(
          new AprilTagVisionIOPhotonReal(
              new VisionConstants("", new Transform3d(), CameraType.LIMELIGHT)));

  PoseEstimate poseEstimate = new PoseEstimate(new Pose3d(), 0.05, 9.0, 3);

  @Test
  void deviationsTest() {
    assertEquals(0.2700, vision.calculateVisionStdDevs(poseEstimate).get(0, 0), 1e-3);
    assertEquals(0.2700, vision.calculateVisionStdDevs(poseEstimate).get(1, 0), 1e-3);
    assertEquals(3.857, vision.calculateVisionStdDevs(poseEstimate).get(2, 0), 1e-3);
  }
}
