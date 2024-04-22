package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.DriveMap.*;
import static frc.robot.Constants.DriveMap.Gyro.GYRO_TYPE;
import static frc.robot.Constants.MODE;
import static frc.robot.Constants.VisionMap.AprilTagVisionMap.LEFT_CAM_CONSTANTS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Config;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Module.ModuleConstants;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.AprilTagVisionIOPhotonReal;
import frc.robot.subsystems.vision.AprilTagVisionIOPhotonSim;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {

  /*
   im an idiot and don't know how to vision, so please only input
   one set of constants per real io, or
   one sim io with multiple sets of constants
  */
  // TODO fix this problem ^^
  protected final AprilTagVision vision =
      Config.Subsystems.VISION_ENABLED
          ? (MODE == Constants.RobotMode.REAL
              ? new AprilTagVision(new AprilTagVisionIOPhotonReal(LEFT_CAM_CONSTANTS))
              : new AprilTagVision(
                  new AprilTagVisionIOPhotonSim(this::getPose, LEFT_CAM_CONSTANTS)))
          : null;
  protected SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  protected Rotation2d rawGyroRotation = new Rotation2d();
  protected SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
      };

  protected SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          kinematics,
          rawGyroRotation,
          lastModulePositions,
          new Pose2d(),
          VecBuilder.fill(0.003, 0.003, 0.0002),
          VecBuilder.fill(0.01, 0.01, 0.01));

  protected final GyroIO gyroIO;
  protected final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  protected final Module[] modules = new Module[4]; // FL, FR, BL, BR

  // add the CANCoder id between the rotator id and offset params
  public static final ModuleConstants frontLeft =
      new ModuleConstants("Front Left", 0, 1, Rotation2d.fromRadians(-Math.PI / 2));
  public static final ModuleConstants frontRight =
      new ModuleConstants("Front Right", 2, 3, Rotation2d.fromRadians(0));
  public static final ModuleConstants backLeft =
      new ModuleConstants("Back Left", 4, 5, Rotation2d.fromRadians(Math.PI));
  public static final ModuleConstants backRight =
      new ModuleConstants("Back Right", 6, 7, Rotation2d.fromRadians(Math.PI / 2));

  protected final SysIdRoutine sysId;

  public SwerveSubsystem(GyroIO gyroIO, ModuleIO[] moduleIOS) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(moduleIOS[0]);
    modules[1] = new Module(moduleIOS[1]);
    modules[2] = new Module(moduleIOS[2]);
    modules[3] = new Module(moduleIOS[3]);

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(MAX_LINEAR_SPEED, DRIVE_BASE_RADIUS, REPLANNING_CONFIG),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        activePath ->
            Logger.recordOutput(
                "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()])));
    PathPlannerLogging.setLogTargetPoseCallback(
        targetPose -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                state -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> {
                  // only 4-mod swerves, please and thank you build team
                  for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));
  }

  public static GyroIO getRealGyro() {
    return GYRO_TYPE == Gyro.GyroType.PIGEON ? new GyroIOPigeon2() : new GyroIONavX();
  }

  public static ModuleIO[] getRealModules() {
    return USING_TALON_DRIVE
        ? new ModuleIO[] {
          new ModuleIOTalonFX(frontLeft),
          new ModuleIOTalonFX(frontRight),
          new ModuleIOTalonFX(backLeft),
          new ModuleIOTalonFX(backRight),
        }
        : new ModuleIO[] {
          VORTEX_DRIVE ? new ModuleIOSparkFlex(frontLeft) : new ModuleIOSparkMax(frontLeft),
          VORTEX_DRIVE ? new ModuleIOSparkFlex(frontRight) : new ModuleIOSparkMax(frontRight),
          VORTEX_DRIVE ? new ModuleIOSparkFlex(backLeft) : new ModuleIOSparkMax(backLeft),
          VORTEX_DRIVE ? new ModuleIOSparkFlex(backRight) : new ModuleIOSparkMax(backRight),
        };
  }

  public static ModuleIO[] getSimModules() {
    return new ModuleIOSim[] {
      new ModuleIOSim(frontLeft.name()),
      new ModuleIOSim(frontRight.name()),
      new ModuleIOSim(backLeft.name()),
      new ModuleIOSim(backRight.name()),
    };
  }

  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Read wheel positions and deltas from each module
    SwerveModulePosition[] modulePositions = getModulePositions();
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      moduleDeltas[moduleIndex] =
          new SwerveModulePosition(
              modulePositions[moduleIndex].distanceMeters
                  - lastModulePositions[moduleIndex].distanceMeters,
              modulePositions[moduleIndex].angle);
      lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }

    // Update gyro angle
    if (gyroInputs.connected) {
      // Use the real gyro angle
      rawGyroRotation = gyroInputs.yawPosition;
    } else {
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist = kinematics.toTwist2d(moduleDeltas);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    // Apply odometry update
    updateOdometry();
    updateVision();
  }

  private void updateOdometry() {
    poseEstimator.update(rawGyroRotation, getModulePositions());
  }

  private void updateVision() {
    vision
        .getProcessedPoseEstimates()
        .forEach(
            timestampedUpdate ->
                poseEstimator.addVisionMeasurement(
                    timestampedUpdate.poseEstimate(),
                    timestampedUpdate.timestamp(),
                    timestampedUpdate.stdDevs()));
  }

  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Returns the module states (turn angles and drive velocities) for all the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all the modules. */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
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
      new Translation2d(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
      new Translation2d(TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0),
      new Translation2d(-TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
      new Translation2d(-TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0),
    };
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }
}
