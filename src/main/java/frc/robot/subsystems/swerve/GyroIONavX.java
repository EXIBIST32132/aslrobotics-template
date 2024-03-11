package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

public class GyroIONavX implements GyroIO {
  private final AHRS navx;

  public GyroIONavX() {
    // default constructor uses the MXP port on the RIO, which is what we want
    navx = new AHRS();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.yawPosition = navx.getRotation2d();
    inputs.connected = navx.isConnected();
    inputs.yawVelocityRadPerSec = navx.getVelocityZ();
  }

  @Override
  public void setYaw(double yaw) {
    navx.setAngleAdjustment(yaw);
  }
}
