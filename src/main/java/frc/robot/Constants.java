// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.util.AprilTagVisionHelpers.TagCountDeviation;
import frc.robot.util.AprilTagVisionHelpers.UnitDeviationParams;
import java.util.List;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final RobotMode MODE = RobotMode.REAL;

  public static enum RobotMode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY,
  }

  public static final class DriveMap {

    public static final boolean USING_TALON_DRIVE = false;
    public static final boolean VORTEX_DRIVE = false && !USING_TALON_DRIVE;

    /** feet per second -> meters per second */
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);

    public static final double TRACK_WIDTH = Units.inchesToMeters(25.0);
    public static final double WHEEL_BASE = Units.inchesToMeters(25.0);
    public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);
    /** radians per second */
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    // Gear ratios for SDS MK4i L2, adjust as necessary
    public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double TURN_GEAR_RATIO = 150.0 / 7.0;
    public static final double WHEEL_RADIUS = Units.inchesToMeters(1.5);
    public static final ReplanningConfig REPLANNING_CONFIG =
        new ReplanningConfig(true, true, 0.6, 0.2);

    public static final class Gyro {

      public enum GyroType {
        PIGEON,
        NAVX,
      }

      public static final GyroType GYRO_TYPE = GyroType.PIGEON;

      public static final int PIGEON_ID = 30;
    }
  }

  public static final class ShooterMap {

    public static final int TOP_FLYWHEEL = 9;
    public static final int BOTTOM_FLYWHEEL = 10;

    // leave it as an explicit division for clarity's sake
    public static final double GEAR_RATIO = 2.0 / 1.0;
  }

  public static final class FieldMap {

    public static final AprilTagFieldLayout APRIL_TAG_LAYOUT_FIELD =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public static final double FIELD_WIDTH_METERS = ((26 * 12) + 11.25) * 25.4 / 1000.0;
    public static final double FIELD_LENGTH_METERS = ((54 * 12) + 3.25) * 25.4 / 1000.0;
  }

  public static final class VisionMap {

    public static enum CameraType {
      OV2311(0.0),
      OV9281(0.0),
      LIMELIGHT(0.0),
      LIMELIGHT_3G(0.0),
      TELEPHOTO_2311(0.0),
      TELEPHOTO_9281(0.0),
      TELEPHOTO_LIMELIGHT(0.0),
      TELEPHOTO_LIMELIGHT_3G(0.0),
      UNKNOWN(Double.MAX_VALUE);

      final double noisyDistance;

      CameraType(double noisyDistance) {
        this.noisyDistance = noisyDistance;
      }

      public double getNoisyDistance() {
        return noisyDistance;
      }

      public static CameraType getFromIndex(int index) {
        return index < 0 ? UNKNOWN : values()[index];
      }
    }

    public record VisionConstants(
        String cameraName, Transform3d robotToCamera, CameraType cameraType) {}

    public static final class GamePieceVisionMap {

      public static final String DETECTOR_LIMELIGHT_NAME = "";

      // 2D stuff
      public static final Transform2d DETECTOR_LIMELIGHT_TO_ROBOT_CENTER =
          new Transform2d(-0.311, -0.276, Rotation2d.fromDegrees(180));

      // 3D stuff
      public static final double DETECTOR_LIMELIGHT_PITCH = -0.349;
      public static final double DETECTOR_LIMELIGHT_ROLL = 0.0;
      public static final double DETECTOR_LIMELIGHT_HEIGHT = 0.453 + 0.066;
    }

    // TODO tune all of these!!
    public static final class AprilTagVisionMap {

      public static final double MAX_DISTANCE_CUTOFF = 5.0;

      public static final boolean LEFT_CAM_ENABLED = true;
      public static final VisionConstants LEFT_CAM_CONSTANTS =
          new VisionConstants("", new Transform3d(), CameraType.TELEPHOTO_9281);

      public static final List<TagCountDeviation> TAG_COUNT_DEVIATION_PARAMS =
          List.of(
              // 1 tag
              new TagCountDeviation(
                  new UnitDeviationParams(.25, .4, .9), new UnitDeviationParams(.5, .7, 1.5)),
              // 2 tags
              new TagCountDeviation(
                  new UnitDeviationParams(.35, .1, .4), new UnitDeviationParams(.5, .7, 1.5)),
              // 3+ tags
              new TagCountDeviation(
                  new UnitDeviationParams(.25, .07, .25), new UnitDeviationParams(.15, 1, 1.5)));
    }
  }
}
