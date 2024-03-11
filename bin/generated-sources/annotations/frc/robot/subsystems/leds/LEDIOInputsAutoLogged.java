package frc.robot.subsystems.leds;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LEDIOInputsAutoLogged extends LEDIO.LEDIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
  }

  @Override
  public void fromLog(LogTable table) {
  }

  public LEDIOInputsAutoLogged clone() {
    LEDIOInputsAutoLogged copy = new LEDIOInputsAutoLogged();
    return copy;
  }
}
