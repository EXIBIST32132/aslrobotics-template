package frc.robot.subsystems.swerve;

import static frc.robot.Constants.DriveMap.*;
import static frc.robot.Constants.DriveMap.MAX_ANGULAR_SPEED;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;

public interface GenericSwerve extends Subsystem {
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds);

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction);

  public Command sysIdDynamic(SysIdRoutine.Direction direction);

  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates();

  /** Returns the module positions (turn angles and drive positions) for all the modules. */
  public SwerveModulePosition[] getModulePositions();

  /** Stops the drive. */
  public default void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public default void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose();

  /** Returns the current odometry rotation. */
  public default Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public void setPose(Pose2d pose);

  public void addVisionMeasurement(Pose2d visionPose, double timestamp);

  /** Returns the maximum linear speed in meters per sec. */
  public default double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public default double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }
}
