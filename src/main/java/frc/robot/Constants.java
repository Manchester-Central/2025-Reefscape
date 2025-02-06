package frc.robot;

import com.chaos131.pid.PIDFValue;
import com.chaos131.pid.PIDValue;
import com.chaos131.robot.ChaosRobot.Mode;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class CanIdentifiers {
    public static final String CTRECANBus = "CTRE bus";

    public static final int FLSpeedCANID = 30;
    public static final int FLAngleCANID = 31;
    public static final int FLAbsoEncoCANID = 32;
    public static final int FRSpeedCANID = 33;
    public static final int FRAngleCANID = 34;
    public static final int FRAbsoEncoCANID = 35;
    public static final int BLSpeedCANID = 39;
    public static final int BLAngleCANID = 40;
    public static final int BLAbsoEncoCANID = 41;
    public static final int BRSpeedCANID = 36;
    public static final int BRAngleCANID = 37;
    public static final int BRAbsoEncoCANID = 38;
    public static final int GyroCANID = 45;
  }

  public static class SwerveConstants {

    public static final InvertedValue InvertedAngle = InvertedValue.Clockwise_Positive;
    public static final SensorDirectionValue InvertedEncoder =
        SensorDirectionValue.Clockwise_Positive;
    public static final double SpeedGearRatio = 7.674;
    public static final double AngleGearRatio = 12.1;
    public static final double SpeedCircumference = 0.1016 * Math.PI;
    public static final double DriverRampRatePeriod = 0.05; // TODO: GET REAL
    public static final double AutonomousRampRatePeriod = 0.05; // TODO: GET REAL
    public static final double MaxFreeSpeedMPS = 4.1605;
    public static final double MaxRotationSpeedRadPS = 12.0; // TODO: GET REAL
    public static final PIDValue DefaultModuleAnglePIDValue = new PIDValue(60.0, 12.0, 0.0);
    public static final PIDFValue DefaultModuleVelocityPIDFValues =
        new PIDFValue(5.0, 0.0, 0.0, 2.19);

    public static class SwerveFLConstants {
      public static final Translation2d ModOffset = new Translation2d(0.3048, 0.3048);
      public static final InvertedValue InvertedSpeed = InvertedValue.CounterClockwise_Positive;
      public static final Rotation2d AngleEncoderOffset = Rotation2d.fromDegrees(0);
    }

    public static class SwerveFRConstants {
      public static final Translation2d ModOffset = new Translation2d(0.3048, -0.3048);
      public static final InvertedValue InvertedSpeed = InvertedValue.Clockwise_Positive;
      public static final Rotation2d AngleEncoderOffset = Rotation2d.fromDegrees(0);
    }

    public static class SwerveBLConstants {
      public static final Translation2d ModOffset = new Translation2d(-0.3048, 0.3048);
      public static final InvertedValue InvertedSpeed = InvertedValue.CounterClockwise_Positive;
      public static final Rotation2d AngleEncoderOffset = Rotation2d.fromDegrees(0);
    }

    public static class SwerveBRConstants {
      public static final Translation2d ModOffset = new Translation2d(-0.3048, -0.3048);
      public static final InvertedValue InvertedSpeed = InvertedValue.Clockwise_Positive;
      public static final Rotation2d AngleEncoderOffset = Rotation2d.fromDegrees(0);
    }
  }

  public static class MidLiftConstants {
    public static class BasePivotConstants {
      public static final Rotation2d MinAngle = Rotation2d.fromDegrees(30);
      public static final Rotation2d MaxAngle = Rotation2d.fromDegrees(90);
      public static final Rotation2d StowAngle = Rotation2d.fromDegrees(40);
      public static final Rotation2d HandoffAngle = Rotation2d.fromDegrees(30);
      public static final Rotation2d ScoreL1Angle = Rotation2d.fromDegrees(54);
      public static final Rotation2d ScoreL2Angle = Rotation2d.fromDegrees(61);
      public static final Rotation2d ScoreL3Angle = Rotation2d.fromDegrees(70);
      public static final Rotation2d ScoreL4Angle = Rotation2d.fromDegrees(70);
    }

    public static class GripperPivotConstants {
      public static final Rotation2d MinAngle = Rotation2d.fromDegrees(-45);
      public static final Rotation2d MaxAngle = Rotation2d.fromDegrees(45);
      public static final Rotation2d StowAngle = Rotation2d.fromDegrees(0);
      public static final Rotation2d HandoffAngle = Rotation2d.fromDegrees(-45);
      public static final Rotation2d ScoreL1Angle = Rotation2d.fromDegrees(-110);
      public static final Rotation2d ScoreL2Angle = Rotation2d.fromDegrees(-117);
      public static final Rotation2d ScoreL3Angle = Rotation2d.fromDegrees(-126);
      public static final Rotation2d ScoreL4Angle = Rotation2d.fromDegrees(51);
    }

    public static class ExtenderConstants {
      public static final double MinLengthMeter = 0.1;
      public static final double MaxLengthMeter = 1.6;
      public static final double StowLengthMeter = 0.3;
      public static final double HandoffLengthMeter = 0.4;
      public static final double ScoreL1LengthMeter = 0.97;
      public static final double ScoreL2LengthMeter = 1.0;
      public static final double ScoreL3LengthMeter = 1.31;
      public static final double ScoreL4LengthMeter = 1.8;
    }

    public static class GripperConstants {}
  }

  public static class IntakeConstants {
    public static final Rotation2d StowAngle = Rotation2d.fromDegrees(100.0);
    public static final Rotation2d DeployAngle = Rotation2d.fromDegrees(10.0);
    public static final Rotation2d HandoffAngle = Rotation2d.fromDegrees(150.0);

    public static final double StowSpeed = 0.0;
    public static final double DeploySpeed = 1.0;
    public static final double HandoffPrepSpeed = 0.0;
    public static final double HandoffSpeed = -1.0;
  }

  public static class GeneralConstants {
    public static final double RobotMassKg = 54.43;
    public static final Mode RobotMode = Mode.REAL;
    public static final Pose2d InitialRobotPose = new Pose2d(7.5, 4, Rotation2d.fromDegrees(180));
  }
}
