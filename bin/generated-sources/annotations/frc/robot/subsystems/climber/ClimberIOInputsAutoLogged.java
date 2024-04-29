package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimberIOInputsAutoLogged extends ClimberIO.ClimberIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("ClimberLeftPositionMeters", climberLeftPositionMeters);
    table.put("ClimberRightPositionMeters", climberRightPositionMeters);
    table.put("ClimberLeftVelocityMetersPerSec", climberLeftVelocityMetersPerSec);
    table.put("ClimberRightVelocityMetersPerSec", climberRightVelocityMetersPerSec);
    table.put("ClimberLeftAppliedVolts", climberLeftAppliedVolts);
    table.put("ClimberRightAppliedVolts", climberRightAppliedVolts);
    table.put("ClimberCurrentAmps", climberCurrentAmps);
    table.put("ClimberSetpointPosition", climberSetpointPosition);
    table.put("OpenLoopStatus", openLoopStatus);
  }

  @Override
  public void fromLog(LogTable table) {
    climberLeftPositionMeters = table.get("ClimberLeftPositionMeters", climberLeftPositionMeters);
    climberRightPositionMeters =
        table.get("ClimberRightPositionMeters", climberRightPositionMeters);
    climberLeftVelocityMetersPerSec =
        table.get("ClimberLeftVelocityMetersPerSec", climberLeftVelocityMetersPerSec);
    climberRightVelocityMetersPerSec =
        table.get("ClimberRightVelocityMetersPerSec", climberRightVelocityMetersPerSec);
    climberLeftAppliedVolts = table.get("ClimberLeftAppliedVolts", climberLeftAppliedVolts);
    climberRightAppliedVolts = table.get("ClimberRightAppliedVolts", climberRightAppliedVolts);
    climberCurrentAmps = table.get("ClimberCurrentAmps", climberCurrentAmps);
    climberSetpointPosition = table.get("ClimberSetpointPosition", climberSetpointPosition);
    openLoopStatus = table.get("OpenLoopStatus", openLoopStatus);
  }

  public ClimberIOInputsAutoLogged clone() {
    ClimberIOInputsAutoLogged copy = new ClimberIOInputsAutoLogged();
    copy.climberLeftPositionMeters = this.climberLeftPositionMeters;
    copy.climberRightPositionMeters = this.climberRightPositionMeters;
    copy.climberLeftVelocityMetersPerSec = this.climberLeftVelocityMetersPerSec;
    copy.climberRightVelocityMetersPerSec = this.climberRightVelocityMetersPerSec;
    copy.climberLeftAppliedVolts = this.climberLeftAppliedVolts;
    copy.climberRightAppliedVolts = this.climberRightAppliedVolts;
    copy.climberCurrentAmps = this.climberCurrentAmps.clone();
    copy.climberSetpointPosition = this.climberSetpointPosition;
    copy.openLoopStatus = this.openLoopStatus;
    return copy;
  }
}
