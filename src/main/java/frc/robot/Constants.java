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

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static java.lang.Math.toRadians;

import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;
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

  public static final RobotMode MODE = RobotMode.SIM;

  public static enum RobotMode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY,
  }

  public static final class DriveMap {

    public static final class GyroMap {

      public static enum GyroType {
        PIGEON,
        NAVX,
        ADIS,
      }

      public static final GyroType GYRO_TYPE = GyroType.PIGEON;

      public static final int PIGEON_ID = 30;
    }

    // TODO refactor into separate constants files in swerve
    public static final boolean USING_TALON_DRIVE = true; // change to using kraken FOC?
    public static final boolean USING_VORTEX_DRIVE = true && !USING_TALON_DRIVE;

    public static final double TRACK_WIDTH = Units.inchesToMeters(23.5);
    public static final double WHEEL_BASE = Units.inchesToMeters(23.5);

    public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);

    /** feet per second -> meters per second */
    public static final double MAX_LINEAR_SPEED = 4 /*Units.feetToMeters(10)*/;
    /** radians per second */
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    public static final double WHEEL_RADIUS = Units.inchesToMeters(1.5);

    // this and below for choreo and pathfinding
    public static final double DRIVE_BASE_MASS = Units.lbsToKilograms(60.0);

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 40;
    public static final int ROTATOR_MOTOR_CURRENT_LIMIT = 20;

    public static final double DRIVE_MOTOR_MAX_TORQUE =
        USING_TALON_DRIVE
            ? DCMotor.getKrakenX60Foc(1).getTorque(DRIVE_MOTOR_CURRENT_LIMIT)
            : (USING_VORTEX_DRIVE
                ? DCMotor.getNeoVortex(1).getTorque(DRIVE_MOTOR_CURRENT_LIMIT)
                : DCMotor.getNEO(1).getTorque(DRIVE_MOTOR_CURRENT_LIMIT));

    public static final double DRIVE_GEAR_RATIO =
        USING_TALON_DRIVE ? (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0) : 4.71;
    public static final double TURN_GEAR_RATIO = USING_TALON_DRIVE ? 150.0 / 7.0 : 9424.0 / 203.0;

    public static final double MAX_LINEAR_ACCELERATION =
        4 * (DRIVE_GEAR_RATIO * DRIVE_MOTOR_MAX_TORQUE) / WHEEL_RADIUS / DRIVE_BASE_MASS;
    public static final double MAX_ANGULAR_ACCELERATION =
        4 * (DRIVE_GEAR_RATIO * DRIVE_MOTOR_MAX_TORQUE) / WHEEL_RADIUS * DRIVE_BASE_RADIUS / 15.0;

    public static final PIDConstants TRANSLATION_CONSTANTS = new PIDConstants(15, 0, 0);
    public static final PIDConstants ROTATION_CONSTANTS = new PIDConstants(15, 0, 0);
    public static final ReplanningConfig REPLANNING_CONFIG =
        new ReplanningConfig(true, true, 3, 0.1);
  }

  public static final class ShooterMap {

    public static final int TOP_FLYWHEEL = 9;
    public static final int BOTTOM_FLYWHEEL = 10;

    // leave it as an explicit division for clarity's sake
    public static final double GEAR_RATIO = 2.0 / 1.0;
  }

  public static final class FieldMap {
    public static enum Coordinates {
      SPEAKER(new Pose2d(FIELD_LENGTH_METERS, 5.55, Rotation2d.fromDegrees(180))),
      AMP(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

      private final Pose2d pose;

      Coordinates(Pose2d pose) {
        this.pose = pose;
      }

      public Pose2d getPose() {
        return pose;
      }
    }

    public static final AprilTagFieldLayout APRIL_TAG_LAYOUT_FIELD =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public static final double FIELD_WIDTH_METERS = ((26 * 12) + 11.25) * 25.4 / 1000.0;
    public static final double FIELD_LENGTH_METERS = ((54 * 12) + 3.25) * 25.4 / 1000.0;
  }

  public static final class VisionMap {

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

      public static CameraType getFromIndex(int index) {
        return index < 0 ? UNKNOWN : values()[index];
      }
    }

    public record VisionConstants(
        String cameraName, Transform3d robotToCamera, CameraType cameraType) {}

    public static final class GamePieceVisionMap {

      public static final String DETECTOR_LIMELIGHT_NAME = "";

      public static final double GAME_PIECE_HEIGHT = Units.inchesToMeters(5.0);

      // 2D stuff
      public static final Transform2d ROBOT_CENTER_TO_DETECTOR_LIMELIGHT_2D =
          new Transform2d(0, 0, Rotation2d.fromDegrees(180));

      // 3D stuff
      public static final double DETECTOR_LIMELIGHT_PITCH = toRadians(20);
      public static final double DETECTOR_LIMELIGHT_ROLL = toRadians(0.0);
      public static final double DETECTOR_LIMELIGHT_HEIGHT = 0.9;

      public static final Transform3d ROBOT_CENTER_TO_DETECTOR_LIMELIGHT_3D =
          new Transform3d(
              ROBOT_CENTER_TO_DETECTOR_LIMELIGHT_2D.getX(),
              ROBOT_CENTER_TO_DETECTOR_LIMELIGHT_2D.getY(),
              DETECTOR_LIMELIGHT_HEIGHT,
              new Rotation3d(
                  DETECTOR_LIMELIGHT_ROLL,
                  DETECTOR_LIMELIGHT_PITCH,
                  ROBOT_CENTER_TO_DETECTOR_LIMELIGHT_2D.getRotation().getRadians()));
    }

    // TODO tune all of these!!
    public static final class AprilTagVisionMap {

      public static final boolean LEFT_CAM_ENABLED = true;
      public static final VisionConstants LEFT_CAM_CONSTANTS =
          new VisionConstants(
              "lefttagcam",
              new Transform3d(
                  new Translation3d(0.306, -0.3, 0.15),
                  new Rotation3d(0, degreesToRadians(-28), degreesToRadians(-30))),
              CameraType.TELEPHOTO_9281);

      public static final boolean RIGHT_CAM_ENABLED = true;
      public static final VisionConstants RIGHT_CAM_CONSTANTS =
          new VisionConstants(
              "righttagcam",
              new Transform3d(
                  new Translation3d(0.306, 0.3, 0.15),
                  new Rotation3d(0, degreesToRadians(-28), degreesToRadians(30))),
              CameraType.TELEPHOTO_9281);

      public static final List<TagCountDeviation> TAG_COUNT_DEVIATION_PARAMS =
          List.of(
              // 1 tag
              new TagCountDeviation(
                  new UnitDeviationParams(0.35, 0.40, 0.90),
                  new UnitDeviationParams(0.50, 0.70, 1.50)),
              // 2 tags
              new TagCountDeviation(
                  new UnitDeviationParams(0.35, 0.1, 0.4),
                  new UnitDeviationParams(0.50, 0.70, 1.50)),
              // 3+ tags
              new TagCountDeviation(
                  new UnitDeviationParams(0.25, 0.07, 0.25),
                  new UnitDeviationParams(0.15, 1.0, 1.50)));

      //      public static final UnitDeviationParams MOVING_DEVIATION_PARAMS =
      //          new UnitDeviationParams(
      //              MOVING_DEVIATION_VELOCITY_MULTIPLIER, MOVING_DEVIATION_EULER_MULTIPLIER, 1);
      public static final double MOVING_DEVIATION_EULER_MULTIPLIER = 0.5;
      public static final double MOVING_DEVIATION_VELOCITY_MULTIPLIER = 0.5;
      public static final double TURNING_DEVIATION_EULER_MULTIPLIER = 0.5;
      public static final double TURNING_DEVIATION_VELOCITY_MULTIPLIER = 0.5;
      public static final double MAX_AMBIGUITY_CUTOFF = 0.05;
    }
  }
}
