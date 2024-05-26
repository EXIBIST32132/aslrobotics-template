package frc.robot.subsystems.swerve;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

public class GyroIOADIS implements GyroIO {
  private final ADIS16470_IMU imu;

  public GyroIOADIS() {
    this.imu = new ADIS16470_IMU();
    imu.calibrate();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.yawPosition = Rotation2d.fromDegrees(imu.getAngle());
    inputs.connected = imu.isConnected();
    // todo this is cursed plz fix
    inputs.yawVelocityRadPerSec =
        Math.toRadians(imu.getAngle() - inputs.yawVelocityRadPerSec) / getFPGATimestamp();
  }

  // don't run this one while turning?
  @Override
  public void setYaw(double yaw) {
    imu.setGyroAngle(ADIS16470_IMU.IMUAxis.kYaw, yaw);
  }
}
