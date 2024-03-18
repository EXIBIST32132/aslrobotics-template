package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Contains the methods that dictate simulated behavior for LEDs. <br>
 * <br>
 * LEDs are likely to be a special case in the AKit paradigm for various reasons, so we'll have to
 * see how things go in simulation. As of now, this class is just a copy of the real version.
 */
public class LEDIOSim implements LEDIO {
  private final AddressableLED led;
  private final AddressableLEDSim sim;
  private final AddressableLEDBuffer buffer;

  public LEDIOSim() {
    led = new AddressableLED(3);
    buffer = new AddressableLEDBuffer(LEDSubsystem.NUM_LEDS);
    led.setLength(buffer.getLength());
    led.start();

    sim = AddressableLEDSim.createForChannel(3);
    setData(buffer);
    sim.setRunning(true);
  }

  /** Write data from buffer to leds */
  @Override
  public void updateInputs(LEDIO.LEDIOInputs inputs) {
    setData(buffer);
  }

  @Override
  public void setLED(int i, Color color) {
    buffer.setLED(i, color);
  }

  private void setData(AddressableLEDBuffer buffer) {
    byte[] data = new byte[buffer.getLength()];

    for (int i = 0; i < buffer.getLength(); i++) {
      data[i] = Byte.parseByte(buffer.getLED8Bit(i).toHexString());
    }

    sim.setData(data);
  }
}
