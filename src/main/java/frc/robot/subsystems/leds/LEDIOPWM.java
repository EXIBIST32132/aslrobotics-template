package frc.robot.subsystems.leds;

import static frc.robot.subsystems.leds.LEDMap.LED_LENGTH;
import static frc.robot.subsystems.leds.LEDMap.LED_PORT;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Contains the methods that dictate real (on-bot) behavior for LEDs. <br>
 * <br>
 * LEDs are likely to be a special case in the AKit paradigm for various reasons, so we'll have to
 * see how things go in simulation. As of now, this class is copied into the real version.
 */
public class LEDIOPWM implements LEDIO {

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  public LEDIOPWM() {
    led = new AddressableLED(LED_PORT);
    buffer = new AddressableLEDBuffer(LED_LENGTH);
    led.setLength(buffer.getLength());
    led.start();
  }

  /** Write data from buffer to leds */
  @Override
  public void updateInputs(LEDIO.LEDIOInputs inputs) {
    led.setData(buffer);
  }

  @Override
  public void setLED(int i, Color color) {
    buffer.setLED(i, color);
  }
}
