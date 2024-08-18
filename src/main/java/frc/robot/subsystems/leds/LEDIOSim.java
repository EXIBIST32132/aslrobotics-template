package frc.robot.subsystems.leds;

import static frc.robot.subsystems.leds.LEDMap.LED_LENGTH;
import static frc.robot.subsystems.leds.LEDMap.LED_PORT;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Contains the methods that dictate simulated behavior for LEDs. <br>
 * <br>
 * Interestingly, you can read AddressableLED data directly from the sim GUI, without a roboRIO – no
 * need for AddressableLEDSim or the byte-conversion nonsense it warrants. Therefore, this is a
 * direct copy of LEDIOPWM.
 */
public class LEDIOSim implements LEDIO {

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  public LEDIOSim() {
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
