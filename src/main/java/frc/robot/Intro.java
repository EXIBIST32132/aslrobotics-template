package frc.robot;

/**
 * Welcome to the template repository for FRC teams 1797 and 1884! <br>
 * <br>
 * Starting the 2024-25 season, we're going to be doing things differently than we have in the past.
 * (Feel free to explore the ASLRobotics and FRC1884 GitHub profiles to see the repos from previous
 * seasons!) <br>
 * <br>
 * The most notable structural switch is that to the AdvantageKit logging paradigm. This has been
 * created and maintained by FRC team 6328 Mechanical Advantage, with the intent to aid both them
 * and other teams in the debugging process via a novel universal logging system. <br>
 * <br>
 * To understand this system, you must grasp one very important "law" about each and every program
 * in FRC: <br>
 * <br>
 * <b>The robot program is a function of all its inputs.</b> <br>
 * <br>
 * This means that for every input you feed into the program, the code will provide a certain,
 * <b>predictable</b> output on the robot, assuming independence from external failures. Akin to the
 * functions you might have learned about in Math, every input will map to exactly one output.
 * <i>(This deterministic approach entails that there must be NO behavior dictated by random chance
 * in any way.)</i> <br>
 * <br>
 * One of the main advantages of this over any other existing logging systems (such as built-in DS
 * logging or even Shuffleboard and other smart dashboards) is that every single input, and
 * therefore every single output and robot situation, can be recorded, <b>even without a full
 * robot</b> using simulated ("simmed") sensors, mechanisms and actuators. <br>
 * <br>
 * Another plus to this is that all behavior can be "replayed" through simulation of the robot using
 * the inputs that had been logged during each match, so that it can be reviewed to help offer some
 * insight into what exactly caused any problems that may have occurred during the match. <br>
 * <br>
 * This system relies on separating each subsystem or mechanism on the robot into at least four
 * separate classes. These are:
 *
 * <ul>
 *   <li>An I/O interface that contains the common behavior for both simmed and real robots. Member
 *       methods can be logged as above automatically, using the {@code @AutoLog} annotation.
 *   <li>A real implementation of the subsystem, containing the hardware and behavioral
 *       implementations that the real robot will use during a match.
 *   <li>A sim implementation of the subsystem, containing sim-specific hardware and behavioral
 *       implementations of the I/O interface.
 *   <li>Perhaps the most familiar of these four, the actual subsystem that contains the
 * </ul>
 *
 * Make sure to check out their webinars, CD threads, and of course, their docs for more
 * information. <br>
 * <br>
 * There have been some other smaller, but still notable changes. The first is the usage of {@link
 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID} and its inheritors as the wrappers for
 * the HID (Human Interface Layer). This is the layer that handles all I/O with the game controllers
 * while the robot is enabled. Previously, we've been writing our own implementations using the much
 * lower-level {@link edu.wpi.first.wpilibj.GenericHID}, simply communicating with the controllers
 * via the internal button ports. While this is essentially what {@code CommandGenericHID} does, our
 * implementation introduced far more bloat into the code than was really necessary, with a dozen
 * files to carry out this interface. With this switch, the I/O layer will of course be easier to
 * navigate, but will also more smoothly integrate with AKit's universal logging structure. <br>
 * <br>
 * Moving up the abstraction layer, a lot of changes have been made to keep new code being written
 * to a minimum. With the introduction of the {@code RobotContainer} class, button bindings,
 * hardware declarations, etc. are largely unified. Additionally, most design choices can be
 * accounted for through very minimal tweaks, down to changing a single config variable. This will
 * be exceedingly helpful during the prototyping stage of build season, as gear ratios, motor
 * controller types, and even entire mechanism archetypes can be rapidly tested. <br>
 * <br>
 * Because of the removal of the CommandMaps, all subsystem commands should now be scheduled <br>
 * <br>
 * <b>Credits</b><br>
 * Massive, massive thank you to FRC teams 5712, 8033, 6443, 6328, 5907, and 1683 for providing
 * their 2023-24 season repositories as inspiration for this project! Without your gracious
 * professionalism, we would not have been able to get this massive project off the ground. Of
 * course, we are also extremely grateful to team 6328 and everyone that has helped maintain the
 * incredible framework that is AdvantageKit, as well as the extensive documentation that it
 * thankfully has.
 */
public final class Intro {

  private Intro() {
    System.out.println("This is an information class!");
  }
}
