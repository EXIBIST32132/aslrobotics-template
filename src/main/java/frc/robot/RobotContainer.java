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

import static frc.robot.Constants.RobotMode;
import static frc.robot.Constants.currentMode;
import static frc.robot.Config.Controllers.getDriverController;
import static frc.robot.Config.Controllers.getOperatorController;
import static frc.robot.Config.Subsystems;
import static frc.robot.subsystems.swerve.MAXSwerve.MAXSwerve.getSparkModules;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Config.Controllers;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSpark;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LEDIOPWM;
import frc.robot.subsystems.leds.LEDIOSim;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.prototypes.Prototypes;
import frc.robot.subsystems.prototypes.Prototypes.PrototypeMotor;
import frc.robot.subsystems.swerve.GenericSwerve;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.MAXSwerve.MAXSwerve;
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
  private final GenericSwerve drive = Subsystems.DRIVETRAIN_ENABLED
    ? (
      currentMode == RobotMode.REAL
        ? new MAXSwerve(new GyroIOPigeon2(), getSparkModules())
        : new MAXSwerve(
          new GyroIO() {
            @Override
            public void updateInputs(GyroIO.GyroIOInputs inputs) {}

            @Override
            public void setYaw(double yaw) {}
          },
          getSparkModules()
        )
    )
    : null;

  private final Flywheel flywheel = Subsystems.SHOOTER_ENABLED
    ? (
      currentMode == RobotMode.REAL
        ? new Flywheel(new FlywheelIOSpark())
        : new Flywheel(new FlywheelIOSim())
    )
    : null;

  private final IntakeSubsystem intake = Subsystems.INTAKE_ENABLED
    ? (
      currentMode == RobotMode.REAL
        ? new IntakeSubsystem(new IntakeIOReal())
        : new IntakeSubsystem(new IntakeIOSim())
    )
    : null;

  private final ClimberSubsystem climber = Subsystems.CLIMBER_ENABLED
    ? (
      currentMode == RobotMode.REAL
        ? new ClimberSubsystem(new ClimberIOReal())
        : new ClimberSubsystem(new ClimberIOSim())
    )
    : null;

  private final LEDSubsystem leds = Subsystems.LEDS_ENABLED
    ? (
      currentMode == RobotMode.REAL
        ? new LEDSubsystem(new LEDIOPWM())
        : new LEDSubsystem(new LEDIOSim())
    )
    : null;

  private final Prototypes prototypes = new Prototypes(
          new PrototypeMotor(1, "Motor 1")
  );

  // Driver controller
  private final CommandXboxController driver = Controllers.DRIVER_ENALBED
    ? (CommandXboxController) getDriverController()
    : null;

  // Operator controller
  private final CommandXboxController operator = Controllers.OPERATOR_ENABLED
    ? (CommandXboxController) getOperatorController()
    : null;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput = new LoggedDashboardNumber(
    "Flywheel Speed",
    1500.0
  );

  private void registerCharacterization() {
    // Set up SysId routines for all subsystems
    autoChooser.addOption(
      "Drive SysId (Quasistatic Forward)",
      drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
    );
    autoChooser.addOption(
      "Drive SysId (Quasistatic Reverse)",
      drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
    );
    autoChooser.addOption(
      "Drive SysId (Dynamic Forward)",
      drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
    );
    autoChooser.addOption(
      "Drive SysId (Dynamic Reverse)",
      drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
    );

    autoChooser.addOption(
      "Flywheel SysId (Quasistatic Forward)",
      flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
    );
    autoChooser.addOption(
      "Flywheel SysId (Quasistatic Reverse)",
      flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
    );
    autoChooser.addOption(
      "Flywheel SysId (Dynamic Forward)",
      flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward)
    );
    autoChooser.addOption(
      "Flywheel SysId (Dynamic Reverse)",
      flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse)
    );
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Set up auto routines
    NamedCommands.registerCommand(
      "Run Flywheel",
      Commands
        .startEnd(
          () -> flywheel.runVelocity(flywheelSpeedInput.get()),
          flywheel::stop,
          flywheel
        )
        .withTimeout(5.0)
    );
    autoChooser =
      new LoggedDashboardChooser<>(
        "Auto Choices",
        AutoBuilder.buildAutoChooser()
      );

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
    drive.setDefaultCommand(
      DriveCommands.joystickDrive(
        drive,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX(),
        () -> -driver.getRightX()
      )
    );
    driver
      .x()
      .onTrue(Commands.runOnce(drive::stopWithX, drive));
    driver
      .b()
      .onTrue(
        Commands
          .runOnce(
            () ->
              drive.setPose(
                new Pose2d(
                  drive.getPose().getTranslation(),
                  new Rotation2d()
                )
              ),
            drive
          )
          .ignoringDisable(true)
      );
    operator
      .a()
      .whileTrue(
        Commands.startEnd(
          () -> flywheel.runVelocity(flywheelSpeedInput.get()),
          flywheel::stop,
          flywheel
        )
      );
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
