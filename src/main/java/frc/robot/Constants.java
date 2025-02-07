package frc.robot;

import com.chaos131.robot.ChaosRobot.Mode;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
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

    public static class SwerveFLConstants {
      public static final Translation2d ModOffset = new Translation2d(0.3048, 0.3048);
      public static final int SpeedCANID = 30;
      public static final int AngleCANID = 31;
      public static final int AbsoEncoCANID = 32;
      public static final InvertedValue InvertedSpeed = InvertedValue.CounterClockwise_Positive;
      public static final Rotation2d AngleEncoderOffset = Rotation2d.fromDegrees(0);
    }

    public static class SwerveFRConstants {
      public static final Translation2d ModOffset = new Translation2d(0.3048, -0.3048);
      public static final int SpeedCANID = 33;
      public static final int AngleCANID = 34;
      public static final int AbsoEncoCANID = 35;
      public static final InvertedValue InvertedSpeed = InvertedValue.Clockwise_Positive;
      public static final Rotation2d AngleEncoderOffset = Rotation2d.fromDegrees(0);
    }

    public static class SwerveBLConstants {
      public static final Translation2d ModOffset = new Translation2d(-0.3048, 0.3048);
      public static final int SpeedCANID = 39;
      public static final int AngleCANID = 40;
      public static final int AbsoEncoCANID = 41;
      public static final InvertedValue InvertedSpeed = InvertedValue.CounterClockwise_Positive;
      public static final Rotation2d AngleEncoderOffset = Rotation2d.fromDegrees(0);
    }

    public static class SwerveBRConstants {
      public static final Translation2d ModOffset = new Translation2d(-0.3048, -0.3048);
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

  public static class GyroConstants {
    public static final int GyroCANID = 45;
  }

  public static class GeneralConstants {
    public static final double RobotMassKg = 54.43;
    public static final Mode RobotMode = Mode.SIM;
    public static final Pose2d InitialRobotPose = new Pose2d(7.5, 4, Rotation2d.fromDegrees(180));
  }

  public static class FieldDimensions {
    // Value taken from Limelight fmap
    public static final double FieldLength = 17.5482504;
    // Value taken from Limelight fmap
    public static final double FieldWidth = 8.0519016;
    // Value taken from field cad
    public static final Transform2d ReefBranchLeft =
        new Transform2d(-0.0536, 0.1643, Rotation2d.fromDegrees(0));
    public static final Transform2d ReefBranchRight =
        new Transform2d(-0.0536, -0.1643, Rotation2d.fromDegrees(0));
    // Value taken from field cad
    public static final double TroughHeightMeters = 0.5175;
    // Value taken from field cad
    public static final double Reef1Meters = 0.7763;
    // Value taken from field cad
    public static final double Reef2Meters = 1.1794;
    // Value taken from field cad
    public static final double Reef3Meters = 1.8287;
    // Value taken from field cad
    public static final double BargeMeters = 1.0;
  }

  public static class RobotDimensions {
    // Includes the Bumpers
    public static final double FrontBackLengthMeters = 0.85;
    // Includes the Bumpers
    public static final double SideSideLengthMeters = 0.85;
    // Buffer space to use between the end effector and an interaction point
    public static final double CoralPlacementMargin = 0.03;
    // Distance from the robot origin to the axle for the Base Pivot
    public static final Transform2d BasePivotOffset =
        new Transform2d(-0.1973, 0.1762, Rotation2d.fromDegrees(0));

    /**
     * Distance from the dynamic lift position to the wrist on the gripper mechanism. If the lift
     * were all the way day, then this would be the distance from the center of the axle of the Base
     * Pivot to the center of the axle of the wrist pivot. This must be rotated by the angle of the
     * Base Pivot at some point in the Forward Kinematics.
     */
    public static final Transform2d LiftToWristOffset =
        new Transform2d(0.1784, 0.4259, Rotation2d.fromDegrees(0));

    /**
     * Distance from the center of the wrist's axle to the point used for the EndEffector
     * calculations Presumably this is just beyond the end of the wheels but should be just past
     * where the Coral is.
     */
    public static final Transform2d WristToEndEffector =
        new Transform2d(0, 0, Rotation2d.fromDegrees(0));
  }
}
