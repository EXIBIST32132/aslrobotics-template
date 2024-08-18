package frc.robot.subsystems.leds;

public final class LEDMap {
  public static final int LED_PORT = 3;
  public static int LED_LENGTH;

  public static final int[] SEGMENTS = new int[] {5, 5, 9};

  static {
    for (int segmentLength : SEGMENTS) LED_LENGTH += segmentLength;
  }
}
