package frc.robot.subsystems.prototypes;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;

import static edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets.kNumberSlider;

public class Prototypes extends SubsystemBase {
  // I LOVE these things! Records are the best things to come to Java.
  public record PrototypeMotor(int id, String name) {}

  private static final String PARENT_TAB_NAME = "Prototypes";
  private static final String CONSTANTS_SUBLAYOUT_NAME = "Constants";
  private static final String PERCENT_SUBLAYOUT_NAME = "Percent control";
  private static final String POSITION_SUBLAYOUT_NAME = "Position control";
  private static final String VELOCITY_SUBLAYOUT_NAME = "Velocity control";

  public Prototypes(PrototypeMotor... motors) {
    for(PrototypeMotor motor : motors) {
      ShuffleboardLayout layout = Shuffleboard.getTab(PARENT_TAB_NAME)
        .getLayout(motor.name, BuiltInLayouts.kList)
        .withSize(2, 6);

      SendableMotor sendableMotor = new SendableMotor(motor.id);
      // PID constants sublayout
      layout
        .getLayout(CONSTANTS_SUBLAYOUT_NAME, BuiltInLayouts.kList)
        .add(sendableMotor);

      // percent (voltage) control sublayout
      layout.getLayout(PERCENT_SUBLAYOUT_NAME, BuiltInLayouts.kList)
        .add(sendableMotor.profiledPercentCommand());
      layout.getLayout(PERCENT_SUBLAYOUT_NAME)
        .add(sendableMotor.toggleEnable());
      layout.getLayout(PERCENT_SUBLAYOUT_NAME)
        .addDouble("Goal percent", () -> sendableMotor.goalPercent)
        .withWidget(kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 1));

      // position control sublayout
      layout
        .getLayout(POSITION_SUBLAYOUT_NAME, BuiltInLayouts.kList)
        .add(sendableMotor.profiledPositionCommand());
      layout
        .getLayout(POSITION_SUBLAYOUT_NAME)
        .add(sendableMotor.positionRangeCommand());
      layout.getLayout(POSITION_SUBLAYOUT_NAME)
        .addDouble("Goal position", () -> sendableMotor.goalPos)
        .withWidget(kNumberSlider)
        .withProperties(Map.of("min", sendableMotor.LOWER_POS_LIMIT, "max", sendableMotor.UPPER_POS_LIMIT));
      layout.getLayout(POSITION_SUBLAYOUT_NAME)
        .addDouble("Actual position", () -> sendableMotor.actualPos);

      // velocity control sublayout
      layout.getLayout(VELOCITY_SUBLAYOUT_NAME)
        .add(sendableMotor.profiledVelocityCommand());
      layout.getLayout(VELOCITY_SUBLAYOUT_NAME)
        .addDouble("Goal velocity", () -> sendableMotor.goalVel)
        .withWidget(kNumberSlider)
        .withProperties(Map.of("min", sendableMotor.LOWER_VEL_LIMIT, "max", sendableMotor.UPPER_VEL_LIMIT));
      layout.getLayout(VELOCITY_SUBLAYOUT_NAME)
        .addDouble("Actual velocity", () -> sendableMotor.actualVel);
    }
  }
}