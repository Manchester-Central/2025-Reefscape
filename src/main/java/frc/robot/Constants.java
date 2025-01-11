package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class SwerveConstants {
    // FL
    public static final double FLModoffsetX = 1;
    public static final double FLModoffsetY = 1;
    public static final int FLSpeedCANID = 30;
    public static final int FLAngleCANID = 31;
    public static final int FLAbsoEncoCANID = 32;
    public static final InvertedValue FLInvertedSpeed = InvertedValue.Clockwise_Positive;
    public static final InvertedValue FLInvertedAngle = InvertedValue.Clockwise_Positive;
    public static final SensorDirectionValue FLInvertedEncoder = SensorDirectionValue.Clockwise_Positive;
    public static final double FLSpeedGearRatio = 1.0;
    public static final double FLAngleGearRatio = 1.0;
    public static final double FLSpeedCircumference = 0.2;
    public static final double FLDriverRampRatePeriod = 1.0;
    public static final double FLAutonomousRampRatePeriod = 1.0;
    public static final Rotation2d FLAngleEncoderOffset = Rotation2d.fromDegrees(0);

    // FR
    public static final double FRModoffsetX = 1;
    public static final double FRModoffsetY = -1;
    public static final int FRSpeedCANID = 33;
    public static final int FRAngleCANID = 34;
    public static final int FRAbsoEncoCANID = 35;
    public static final InvertedValue FRInvertedSpeed = InvertedValue.Clockwise_Positive;
    public static final InvertedValue FRInvertedAngle = InvertedValue.Clockwise_Positive;
    public static final SensorDirectionValue FRInvertedEncoder = SensorDirectionValue.Clockwise_Positive;
    public static final double FRSpeedGearRatio = 1.0;
    public static final double FRAngleGearRatio = 1.0;
    public static final double FRSpeedCircumference = 0.2;
    public static final double FRDriverRampRatePeriod = 1.0;
    public static final double FRAutonomousRampRatePeriod = 1.0;
    public static final Rotation2d FRAngleEncoderOffset = Rotation2d.fromDegrees(0);

    // BL
    public static final double BLModoffsetX = -1;
    public static final double BLModoffsetY = 1;
    public static final int BLSpeedCANID = 39;
    public static final int BLAngleCANID = 40;
    public static final int BLAbsoEncoCANID = 41;
    public static final InvertedValue BLInvertedSpeed = InvertedValue.Clockwise_Positive;
    public static final InvertedValue BLInvertedAngle = InvertedValue.Clockwise_Positive;
    public static final SensorDirectionValue BLInvertedEncoder = SensorDirectionValue.Clockwise_Positive;
    public static final double BLSpeedGearRatio = 1.0;
    public static final double BLAngleGearRatio = 1.0;
    public static final double BLSpeedCircumference = 0.2;
    public static final double BLDriverRampRatePeriod = 1.0;
    public static final double BLAutonomousRampRatePeriod = 1.0;
    public static final Rotation2d BLAngleEncoderOffset = Rotation2d.fromDegrees(0);

    // BR
    public static final double BRModoffsetX = -1;
    public static final double BRModoffsetY = -1;
    public static final int BRSpeedCANID = 36;
    public static final int BRAngleCANID = 37;
    public static final int BRAbsoEncoCANID = 38;
    public static final InvertedValue BRInvertedSpeed = InvertedValue.Clockwise_Positive;
    public static final InvertedValue BRInvertedAngle = InvertedValue.Clockwise_Positive;
    public static final SensorDirectionValue BRInvertedEncoder = SensorDirectionValue.Clockwise_Positive;
    public static final double BRSpeedGearRatio = 1.0;
    public static final double BRAngleGearRatio = 1.0;
    public static final double BRSpeedCircumference = 0.2;
    public static final double BRDriverRampRatePeriod = 1.0;
    public static final double BRAutonomousRampRatePeriod = 1.0;
    public static final Rotation2d BRAngleEncoderOffset = Rotation2d.fromDegrees(0);
  }

  public static class GyroConstants{
    public static final int GyroCANID = 0;
  }
}
