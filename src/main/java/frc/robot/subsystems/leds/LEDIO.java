package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;
import org.littletonrobotics.junction.AutoLog;

/**
 * This class contains "common" behavior between real and simulated LEDs. Stuff we want to see in
 * both modes, like setting the whole strip to one color or setting individual LEDs, go in here.
 */
public interface LEDIO {
  /** LEDs don't read any sensors, so this inner class should be left empty. */
  @AutoLog
  class LEDIOInputs {
    // No sensors so no inputs
  }

  /**
   * Updates the LEDs using any inputs provided. This runs once every loop, whether in sim mode or
   * real.
   *
   * @param inputs the inputs to read. See the note at {@link LEDIOInputs} to understand why
   *     inheritors likely won't use this param.
   */
  void updateInputs(LEDIOInputs inputs);

  void setLED(int i, Color color);

  default void setColor(Color color) {
    for (int i = 0; i < LEDSubsystem.NUM_LEDS; i++) {
      setLED(i, color);
    }
  }
}
