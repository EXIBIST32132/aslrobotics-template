package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim implements PivotIO {

  private SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(2), 30, 1, 0.5, 0, Units.degreesToRadians(110), true, 0);

  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(PivotIOInputsAutoLogged inputs) {
    if (closedLoop) {
      appliedVolts = MathUtil.clamp(pid.calculate(sim.getAngleRads()) + ffVolts, -12.0, 12.0);
      sim.setInputVoltage(appliedVolts);
    }
    sim.update(0.02);

    inputs.pivotPositionRad = sim.getAngleRads();
    inputs.pivotVelocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.pivotAppliedVolts = appliedVolts;
    inputs.pivotCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void setPosition(double angleRads, double ffVolts) {
    closedLoop = true;
    pid.setSetpoint(angleRads);
    this.ffVolts = ffVolts;
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
