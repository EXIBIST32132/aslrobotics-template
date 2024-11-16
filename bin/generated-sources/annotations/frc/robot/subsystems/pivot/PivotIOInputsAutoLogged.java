package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class PivotIOInputsAutoLogged extends PivotIO.PivotIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("LeaderConnected", leaderConnected);
    table.put("FollowerConnected", followerConnected);
    table.put("LeaderPositionRad", leaderPositionRad);
    table.put("LeaderTargetPositionRad", leaderTargetPositionRad);
    table.put("EncoderPositionRad", encoderPositionRad);
    table.put("LeaderVelocityRadPerSec", leaderVelocityRadPerSec);
    table.put("LeaderAppliedVolts", leaderAppliedVolts);
    table.put("LeaderCurrentAmps", leaderCurrentAmps);
    table.put("FollowerPositionRad", followerPositionRad);
    table.put("FollowerVelocityRadPerSec", followerVelocityRadPerSec);
  }

  @Override
  public void fromLog(LogTable table) {
    leaderConnected = table.get("LeaderConnected", leaderConnected);
    followerConnected = table.get("FollowerConnected", followerConnected);
    leaderPositionRad = table.get("LeaderPositionRad", leaderPositionRad);
    leaderTargetPositionRad = table.get("LeaderTargetPositionRad", leaderTargetPositionRad);
    encoderPositionRad = table.get("EncoderPositionRad", encoderPositionRad);
    leaderVelocityRadPerSec = table.get("LeaderVelocityRadPerSec", leaderVelocityRadPerSec);
    leaderAppliedVolts = table.get("LeaderAppliedVolts", leaderAppliedVolts);
    leaderCurrentAmps = table.get("LeaderCurrentAmps", leaderCurrentAmps);
    followerPositionRad = table.get("FollowerPositionRad", followerPositionRad);
    followerVelocityRadPerSec = table.get("FollowerVelocityRadPerSec", followerVelocityRadPerSec);
  }

  public PivotIOInputsAutoLogged clone() {
    PivotIOInputsAutoLogged copy = new PivotIOInputsAutoLogged();
    copy.leaderConnected = this.leaderConnected;
    copy.followerConnected = this.followerConnected;
    copy.leaderPositionRad = this.leaderPositionRad;
    copy.leaderTargetPositionRad = this.leaderTargetPositionRad;
    copy.encoderPositionRad = this.encoderPositionRad;
    copy.leaderVelocityRadPerSec = this.leaderVelocityRadPerSec;
    copy.leaderAppliedVolts = this.leaderAppliedVolts;
    copy.leaderCurrentAmps = this.leaderCurrentAmps;
    copy.followerPositionRad = this.followerPositionRad;
    copy.followerVelocityRadPerSec = this.followerVelocityRadPerSec;
    return copy;
  }
}
