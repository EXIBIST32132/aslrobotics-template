package frc.robot.subsystems;

import static com.pathplanner.lib.auto.NamedCommands.registerCommand;
import static frc.robot.GlobalConstants.MODE;
import static frc.robot.subsystems.Superstructure.SuperStates.IDLING;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Config;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.feeder.FeederIOReal;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSpark;
import frc.robot.subsystems.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LEDIOPWM;
import frc.robot.subsystems.leds.LEDIOSim;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.pivot.PivotIOReal;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Superstructure extends SubsystemBase {
  /**
   * For use during autonomous and the Superstructure's periodic. The action (non-idle) entries end
   * in "ing" intentionally – if the robot is not in the state of actively transitioning between
   * states, it's idling.
   */
  private final Supplier<Pose2d> drivePoseSupplier;

  private final PivotSubsystem pivot =
      Config.Subsystems.PIVOT_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new PivotSubsystem(new PivotIOReal())
              : new PivotSubsystem(new PivotIOSim()))
          : null;

  private final FlywheelSubsystem flywheel =
      Config.Subsystems.SHOOTER_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new FlywheelSubsystem(new FlywheelIOSpark())
              : new FlywheelSubsystem(new FlywheelIOSim()))
          : null;

  private final IntakeSubsystem intake =
      Config.Subsystems.INTAKE_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new IntakeSubsystem(new IntakeIOReal())
              : new IntakeSubsystem(new IntakeIOSim()))
          : null;

  private final FeederSubsystem feeder =
      Config.Subsystems.FEEDER_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new FeederSubsystem(new FeederIOReal())
              : new FeederSubsystem(new FeederIOSim()))
          : null;

  private final ClimberSubsystem climber =
      Config.Subsystems.CLIMBER_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new ClimberSubsystem(new ClimberIOReal())
              : new ClimberSubsystem(new ClimberIOSim()))
          : null;

  private final LEDSubsystem leds =
      Config.Subsystems.LEDS_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new LEDSubsystem(new LEDIOPWM())
              : new LEDSubsystem(new LEDIOSim()))
          : null;

  public Superstructure(Supplier<Pose2d> drivePoseSupplier) {
    this.drivePoseSupplier = drivePoseSupplier;
  }

  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  public void registerAutoCommands() {
    if (flywheel != null) {
      registerCommand(
          "Run Flywheel",
          startEnd(() -> flywheel.runVelocity(flywheelSpeedInput::get), flywheel::stop)
              .withTimeout(5.0));
    }
  }

  public static enum SuperStates {
    IDLING,
    INTAKING,
    PREP_SHOT,
    SHOOT
  }

  /**
   * For use during teleop to modify the current SuperState. Each one requests a state in {@link
   * SuperStates}. REQ_NONE is the absence of
   */
  public static enum StateRequests {
    REQ_NONE,
    REQ_IDLE,
    REQ_INTAKE,
    REQ_SHOOT
  }

  private SuperStates currentState = IDLING;

  public Command setSuperStateCmd(SuperStates stateRequest) {
    return Commands.run(() -> currentState = stateRequest);
  }

  @Override
  public void periodic() {
    switch (currentState) {
      case IDLING -> {
        if (intake != null) intake.stop();
        if (feeder != null) feeder.runVolts(0);
        if (flywheel != null) flywheel.runVelocityCmd(() -> 0);
        if (pivot != null) pivot.stop();
        if (leds != null)
          leds.setRunAlongCmd(
              () -> AllianceFlipUtil.shouldFlip() ? Color.kRed : Color.kBlue,
              () -> Color.kBlack,
              5,
              1);
      }
      case INTAKING -> {
        if (intake != null) intake.fast();
        if (feeder != null) feeder.runVolts(0.5);
      }
      case PREP_SHOT -> {
        flywheel.runVelocityCmd(() -> 2000);
        pivot.setPosition(() -> 0);
      }
      case SHOOT -> {
        feeder.runVolts(5);
        pivot.setPosition(() -> -2);
        flywheel.runVelocityCmd(() -> 2000);
      }
    }
  }

  public Command setLEDBlinkingCmd(Color onColor, Color offColor, double frequency) {
    if (leds != null) return leds.setBlinkingCmd(onColor, offColor, frequency);
    else return null;
  }

  public Trigger shooterVelocityGreater() {
    return new Trigger(() -> flywheel.getVelocityRPM() > 1200);
  }

  public void registerSuperstructureCharacterization(
      Supplier<LoggedDashboardChooser<Command>> autoChooser) {
    // Set up SysId routines for all subsystems
    if (flywheel != null) {
      autoChooser
          .get()
          .addOption(
              "Flywheel SysId (Quasistatic Forward)",
              flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooser
          .get()
          .addOption(
              "Flywheel SysId (Quasistatic Reverse)",
              flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooser
          .get()
          .addOption(
              "Flywheel SysId (Dynamic Forward)",
              flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
      autoChooser
          .get()
          .addOption(
              "Flywheel SysId (Dynamic Reverse)",
              flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
  }
}
