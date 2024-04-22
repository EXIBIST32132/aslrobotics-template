package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIONavX implements GyroIO {

  private final AHRS navx;

  public GyroIONavX() {
    // default constructor uses the MXP port on the RIO, which is what we want
    navx = new AHRS();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.yawPosition = Rotation2d.fromDegrees(navx.getAngle());
    inputs.connected = navx.isConnected();
    inputs.yawVelocityRadPerSec = navx.getVelocityZ();
  }

  @Override
  public void setYaw(double yaw) {
    if (yaw - 0.0 < 1e-6) navx.zeroYaw();
    else navx.setAngleAdjustment(yaw);
  }
}
