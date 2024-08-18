package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.Logger;

public class PivotVisualizer {

  private final String name;

  public PivotVisualizer(String name) {
    this.name = name;
  }

  public void update(double angle) {
    Pose3d pivot = new Pose3d(-0.25, 0, 0.155, new Rotation3d(0, -angle, 0));
    Logger.recordOutput("Pivot/" + name, pivot);
  }
}
