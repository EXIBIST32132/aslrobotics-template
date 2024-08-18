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

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

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
import edu.wpi.first.wpilibj2.command.Command;
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
    return run(
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
        .alongWith(runOnce(() -> Logger.recordOutput("setpoint", pid.getSetpoint())));
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

  public static Command orbitWithDynamicTolerance(
      SwerveSubsystem swerveSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Pose2d> targetPose) {
    return orbitWithDynamicTolerance(
        swerveSubsystem,
        xSupplier,
        ySupplier,
        () ->
            -targetPose
                    .get()
                    .getTranslation()
                    .minus(swerveSubsystem.getPose().getTranslation())
                    .getAngle()
                    .getDegrees()
                + swerveSubsystem.getRotation().getDegrees(),
        () ->
            targetPose
                .get()
                .getTranslation()
                .getDistance(swerveSubsystem.getPose().getTranslation()));
  }

  public static Command orbitWithDynamicTolerance(
      SwerveSubsystem swerveSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationOverride,
      DoubleSupplier distance) {
    PIDController omegaPID = new PIDController(0.025, 0, 0);
    omegaPID.setTolerance(0.25);
    omegaPID.enableContinuousInput(-180, 180);

    return joystickDrive(
            swerveSubsystem,
            xSupplier,
            ySupplier,
            () -> omegaPID.calculate(0, orbitWrap(rotationOverride)))
        .alongWith(
            runOnce(() -> omegaPID.setTolerance(Math.exp(-1 * (distance.getAsDouble()) + 1.25)))
                .repeatedly());
  }

  // to stop jittering if facing opposite direction
  // TODO fix
  private static double orbitWrap(DoubleSupplier rotationOverride) {
    if (Math.abs(rotationOverride.getAsDouble() - 90) < 5)
      return -(rotationOverride.getAsDouble() - 45);
    else if (Math.abs(rotationOverride.getAsDouble() + 90) < 5)
      return -(rotationOverride.getAsDouble() + 45);
    else if (rotationOverride.getAsDouble() > 90) return -(rotationOverride.getAsDouble() - 90);
    else if (rotationOverride.getAsDouble() < -90) return -(rotationOverride.getAsDouble() + 90);
    else return -rotationOverride.getAsDouble();
  }
}
