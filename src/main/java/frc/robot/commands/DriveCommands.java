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

package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {

  private static final double DEADBAND = 0.1;

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      SwerveSubsystem driveSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          driveSubsystem.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * driveSubsystem.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * driveSubsystem.getMaxLinearSpeedMetersPerSec(),
                  omega * driveSubsystem.getMaxAngularSpeedRadPerSec(),
                  isFlipped
                      ? driveSubsystem.getRotation().plus(new Rotation2d(Math.PI))
                      : driveSubsystem.getRotation()));
        },
        driveSubsystem);
  }

  public static Command driveOnTargetLock(
      SwerveSubsystem driveSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> targetRotation) {
    PIDController pid = new PIDController(0.025, 0, 0.001);
    pid.setTolerance(0.001);
    pid.enableContinuousInput(-180, 180);
    return joystickDrive(
            driveSubsystem,
            xSupplier,
            ySupplier,
            () ->
                pid.calculate(
                    driveSubsystem.getRotation().getDegrees(), targetRotation.get().getDegrees()))
        .alongWith(Commands.runOnce(() -> Logger.recordOutput("setpoint", pid.getSetpoint())));
  }

  public static Command manualOverrideAutoDrive(
      SwerveSubsystem driveSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      String pathName) {
    return driveSubsystem
        .pathFindThenFollowPathCommand(() -> PathPlannerPath.fromPathFile(pathName))
        .until(
            () ->
                xSupplier.getAsDouble() > DEADBAND
                    || ySupplier.getAsDouble() > DEADBAND
                    || omegaSupplier.getAsDouble() > DEADBAND);
  }

  public static Command orbit(
      SwerveSubsystem swerveSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationOverride) {
    PIDController omegaPID = new PIDController(0.05, 0, 0.005);
    omegaPID.setTolerance(1.5);
    omegaPID.enableContinuousInput(-180, 180);

    return joystickDrive(
        swerveSubsystem,
        xSupplier,
        ySupplier,
        () -> omegaPID.calculate(0, -rotationOverride.getAsDouble()));
  }
}
