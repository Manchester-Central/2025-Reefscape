package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class SwerveConstants {

    public static final InvertedValue InvertedAngle = InvertedValue.Clockwise_Positive;
    public static final SensorDirectionValue InvertedEncoder =
        SensorDirectionValue.Clockwise_Positive;
    public static final double SpeedGearRatio = 6.55; // TODO: GET REAL
    public static final double AngleGearRatio = 144.0 / 14.0; // TODO: GET REAL
    public static final double SpeedCircumference = 0.1016 * Math.PI; // TODO: GET REAL
    public static final double DriverRampRatePeriod = 0.05; // TODO: GET REAL
    public static final double AutonomousRampRatePeriod = 0.65; // TODO: GET REAL

    public static class SwerveFLConstants {
      public static final Translation2d ModOffset = new Translation2d(0.3, 0.3); // TODO: GET REAL
      public static final int SpeedCANID = 30;
      public static final int AngleCANID = 31;
      public static final int AbsoEncoCANID = 32;
      public static final InvertedValue InvertedSpeed = InvertedValue.CounterClockwise_Positive;
      public static final Rotation2d AngleEncoderOffset = Rotation2d.fromDegrees(0);
    }

    public static class SwerveFRConstants {
      public static final Translation2d ModOffset = new Translation2d(0.3, -0.3); // TODO: GET REAL
      public static final int SpeedCANID = 33;
      public static final int AngleCANID = 34;
      public static final int AbsoEncoCANID = 35;
      public static final InvertedValue InvertedSpeed = InvertedValue.Clockwise_Positive;
      public static final Rotation2d AngleEncoderOffset = Rotation2d.fromDegrees(0);
    }

    public static class SwerveBLConstants {
      public static final Translation2d ModOffset = new Translation2d(-0.3, 0.3); // TODO: GET REAL
      public static final int SpeedCANID = 39;
      public static final int AngleCANID = 40;
      public static final int AbsoEncoCANID = 41;
      public static final InvertedValue InvertedSpeed = InvertedValue.CounterClockwise_Positive;
      public static final Rotation2d AngleEncoderOffset = Rotation2d.fromDegrees(0);
    }

    public static class SwerveBRConstants {
      public static final Translation2d ModOffset = new Translation2d(-0.3, -0.3); // TODO: GET REAL
      public static final int SpeedCANID = 36;
      public static final int AngleCANID = 37;
      public static final int AbsoEncoCANID = 38;
      public static final InvertedValue InvertedSpeed = InvertedValue.Clockwise_Positive;
      public static final Rotation2d AngleEncoderOffset = Rotation2d.fromDegrees(0);
    }
  }

  public static class MidLiftConstants {
    public static class BasePivotConstants {
      public static final Rotation2d MinAngle = Rotation2d.fromDegrees(30);
      public static final Rotation2d MaxAngle = Rotation2d.fromDegrees(90);
      public static final Rotation2d StowAngle = Rotation2d.fromDegrees(40);
    }

    public static class GripperPivotConstants {
      public static final Rotation2d MinAngle = Rotation2d.fromDegrees(-45);
      public static final Rotation2d MaxAngle = Rotation2d.fromDegrees(45);
      public static final Rotation2d StowAngle = Rotation2d.fromDegrees(0);
    }

    public static class ExtenderConstants {
      public static final double MinLengthMeter = 0.1;
      public static final double MaxLengthMeter = 1.6;
      public static final double StowLengthMeter = 0.3;
    }

    public static class GripperConstants {}
  }

  public static class GyroConstants {
    public static final int GyroCANID = 0;
  }
}
