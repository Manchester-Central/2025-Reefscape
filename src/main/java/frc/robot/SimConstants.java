package frc.robot;

/**
 * This class holds all of our 2025 constants that are overrides for simulation.
 */
public final class SimConstants {

  /** This contains sim constants for our Base Pivot. */
  public static class SimBasePivotConstants {

    public static final double kP = 400.0;
    public static final double kI = 10.00;
    public static final double kD = 0.0;
    
    public static final double canCoderOffsetDegrees = 90;
  }

  /** This contains sim constants for our Gripper Pivot. */
  public static class SimGripperPivotConstants {

    public static final double kP = 10.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double canCoderOffsetDegrees = 0;
  }

  /** This contains sim constants for our Extender. */
  public static class SimExtenderConstants {

    // Slot 0 Configs
    public static final double kP = 20.0;
    public static final double kI = 1.0;
    public static final double kD = 5.0;

  }

}
