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

import static com.pathplanner.lib.auto.NamedCommands.registerCommand;
import static com.pathplanner.lib.path.PathPlannerPath.fromPathFile;
import static edu.wpi.first.wpilibj2.command.Commands.startEnd;
import static frc.robot.Config.Controllers.*;
import static frc.robot.Config.Subsystems;
import static frc.robot.Config.Subsystems.DRIVETRAIN_ENABLED;
import static frc.robot.Config.Subsystems.SHOOTER_ENABLED;
import static frc.robot.GlobalConstants.FieldMap.Coordinates.AMP;
import static frc.robot.GlobalConstants.FieldMap.Coordinates.SPEAKER;
import static frc.robot.GlobalConstants.MODE;
import static frc.robot.GlobalConstants.RobotMode;
import static frc.robot.subsystems.swerve.SwerveSubsystem.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Config.Controllers;
import frc.robot.OI.DriverMap;
import frc.robot.OI.OperatorMap;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSpark;
import frc.robot.subsystems.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LEDIOPWM;
import frc.robot.subsystems.leds.LEDIOSim;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.pivot.PivotIOReal;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.prototypes.Prototypes;
import frc.robot.subsystems.prototypes.Prototypes.PrototypeMotor;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GamePieceVisualizer;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final SwerveSubsystem drive =
      DRIVETRAIN_ENABLED
          ? (MODE == RobotMode.REAL
              ? new SwerveSubsystem(getRealGyro(), getRealModules())
              : new SwerveSubsystem(new GyroIO() {}, getSimModules()))
          : null;

  private final PivotSubsystem pivot =
      Subsystems.PIVOT_ENABLED
          ? (MODE == RobotMode.REAL
              ? new PivotSubsystem(new PivotIOReal())
              : new PivotSubsystem(new PivotIOSim()))
          : null;

  private final FlywheelSubsystem flywheel =
      Subsystems.SHOOTER_ENABLED
          ? (MODE == RobotMode.REAL
              ? new FlywheelSubsystem(new FlywheelIOSpark())
              : new FlywheelSubsystem(new FlywheelIOSim()))
          : null;

  private final IntakeSubsystem intake =
      Subsystems.INTAKE_ENABLED
          ? (MODE == RobotMode.REAL
              ? new IntakeSubsystem(new IntakeIOReal())
              : new IntakeSubsystem(new IntakeIOSim()))
          : null;

  private final ClimberSubsystem climber =
      Subsystems.CLIMBER_ENABLED
          ? (MODE == RobotMode.REAL
              ? new ClimberSubsystem(new ClimberIOReal())
              : new ClimberSubsystem(new ClimberIOSim()))
          : null;

  private final LEDSubsystem leds =
      Subsystems.LEDS_ENABLED
          ? (MODE == RobotMode.REAL
              ? new LEDSubsystem(new LEDIOPWM())
              : new LEDSubsystem(new LEDIOSim()))
          : null;

  private final Prototypes prototypes =
      Subsystems.PROTOTYPES_ENABLED ? new Prototypes(new PrototypeMotor(1, "Motor 1")) : null;

  // Driver controller
  private final DriverMap driver = Controllers.DRIVER_ENALBED ? getDriverController() : null;

  // Operator controller
  private final OperatorMap operator =
      Controllers.OPERATOR_ENABLED ? getOperatorController() : null;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  private void registerCharacterization() {
    // Set up SysId routines for all subsystems
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Forward)",
        flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Reverse)",
        flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Forward)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Reverse)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Set up auto routines
    registerCommand(
        "Run Flywheel",
        startEnd(() -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel)
            .withTimeout(5.0));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    registerCharacterization();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    registerDrivetrain();
    registerFlywheel();

    operator
        .pivotToSpeaker()
        .or(
            new Trigger(
                    () ->
                        drive
                                .getPose()
                                .getTranslation()
                                .getDistance(SPEAKER.getPose().getTranslation())
                            < Units.feetToMeters(25))
                .and(driver.alignToSpeaker()))
        .whileTrue(
            pivot.run(
                () ->
                    pivot.setPosition(
                        () ->
                            Math.atan2(
                                2.1,
                                drive
                                    .getPose()
                                    .getTranslation()
                                    .getDistance(SPEAKER.getPose().getTranslation())))));

    pivot.setDefaultCommand(pivot.run(() -> pivot.setPosition(() -> 0)));

    leds.setDefaultCommand(
        leds.setRunAlongCmd(
            () -> AllianceFlipUtil.shouldFlip() ? Color.kRed : Color.kBlue,
            () -> Color.kBlack,
            5,
            1));

    driver.testButton().onTrue(driver.rumble().withTimeout(1.0));

    new Trigger(() -> Math.abs(drive.getNoteOffset()) < 5 && drive.getNoteOffset() != 0)
        .whileTrue(leds.setBlinkingCmd(Color.kYellow, Color.kBlack, 5));

    GamePieceVisualizer.setRobotPoseSupplier(drive::getPose);
    GamePieceVisualizer.setTargetSupplier(
        () ->
            new Pose3d(
                new Translation3d(
                    SPEAKER.getPose().getTranslation().getX(),
                    SPEAKER.getPose().getTranslation().getY(),
                    2.1),
                new Rotation3d()));
    operator
        .shoot()
        .and(new Trigger(() -> flywheel.getVelocityRPM() > 1200))
        .onTrue(GamePieceVisualizer.shoot(() -> 0.5, pivot::getPosition));
  }

  private void registerDrivetrain() {
    if (DRIVETRAIN_ENABLED && DRIVER_ENALBED) {
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive, driver.getXAxis(), driver.getYAxis(), driver.getRotAxis()));

      /*driver
          //          .start()
          .leftBumper()
          .whileTrue(
              DriveCommands.manualOverrideAutoDrive(
                  drive,
                  () -> -driver.getLeftY(),
                  () -> -driver.getLeftX(),
                  () -> -driver.getRightX(),
                  "Speaker"));
      driver
          //          .back()
          .rightBumper()
          .whileTrue(
              DriveCommands.manualOverrideAutoDrive(
                  drive,
                  () -> -driver.getLeftY(),
                  () -> -driver.getLeftX(),
                  () -> -driver.getRightX(),
                  "Source"));*/
      //      driver.rightBumper().whileTrue(drive.followPathCommand(() -> fromPathFile("Example
      // Path")));

      driver
          .alignToSpeaker()
          .whileTrue(
              DriveCommands.orbitWithDynamicTolerance(
                  drive, driver.getXAxis(), driver.getYAxis(), SPEAKER::getPose));

      //      driver.x().onTrue(runOnce(drive::stopWithX, drive));
      //      driver
      //        .b()
      //        .onTrue(
      //          runOnce(
      //            () ->
      //              drive.setPose(
      //                new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
      //            drive)
      //            .ignoringDisable(true));

      new Trigger(
              () ->
                  drive.getPose().getTranslation().getDistance(AMP.getPose().getTranslation())
                      <= 2.5)
          .and(driver.pathToAmp())
          .onTrue(drive.pathFindThenFollowPathCommand(() -> fromPathFile("Amp")));

      driver
          .alignToGamePiece()
          .and(new Trigger(drive::hasNote))
          .whileTrue(
              DriveCommands.orbitWithDynamicTolerance(
                  drive, driver.getXAxis(), driver.getYAxis(), drive::getNoteOffset, () -> 2.5));
    }
  }

  private void registerFlywheel() {
    if (SHOOTER_ENABLED && OPERATOR_ENABLED) {
      operator
          .shoot()
          .whileTrue(
              startEnd(
                  () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
