package frc.robot;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.chaos131.pid.PIDFValue;
import com.chaos131.pid.PIDValue;
import com.chaos131.robot.ChaosRobot.Mode;
import com.chaos131.vision.CameraSpecs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.lift.LiftPose;

/** This class holds all of our 2025 constants. */
public final class Constants {

  /** This has contants that can be used throughout the robot. */
  public static class GeneralConstants {
    public static final double RobotMassKg = 54.43;
    public static final Mode RobotMode = Mode.REAL;
    public static final Pose2d InitialRobotPose = new Pose2d(7.5, 4, Rotation2d.fromDegrees(180));
  }

  /** This contains constants needed for setting up our controllers. */
  public static class OperatorConstants {
    public static final int DriverControllerPort = 0;
    public static final int OperatorControllerPort = 1;
    public static final int SimulationControllerPort = 2;
    public static final int TesterControllerPort = 3;
  }

  /** This contains all of our constants for CAN IDs and Can Bus Names. */
  public static class CanIdentifiers {
    public static final String CTRECANBus = "CTRE bus";

    // Swerve (30s & 40s)
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

    // Base Pivot (20s)
    public static final int BasePivotMotorCANID = 20;
    public static final int BasePivotCANcoderCANID = 21;

    // Extender (50s)
    public static final int ExtenderMotorCANID = 51;

    // Gripper Pivot (60s)
    public static final int GripperPivotMotorCANID = 60; // TODO: set on robot
    public static final int GripperPivotCANCoderCANID = 61; // TODO: set on robot

    // Gripper (70s)
    public static final int GripperCoralMotorCANID = 13; // TODO: set on robot
    // public static final int GripperAlgaeMotorCANID = 71; // TODO: set on robot

    // Intake (80s) RIP 2025-2025 for now for now
    public static final int IntakeMotor1CANID = 80; // TODO: set on robot
    public static final int IntakeMotorBCANID = 81; // TODO: set on robot
  }

  /** This contains constants for all our IO ports on the RIO. */
  public static class IoPortsConstants {
    public static final int CoralChannelID = 1;
    public static final int ExtenderMinimumChannelID = 4;
  }

  /** This contains constants for setting up our swerve drive. */
  public static class SwerveConstants {
    public static final boolean AcceptVisionUpdates = true;

    public static final InvertedValue InvertedAngle = InvertedValue.CounterClockwise_Positive;
    public static final SensorDirectionValue InvertedEncoder =
        SensorDirectionValue.CounterClockwise_Positive;
    public static final double SpeedGearRatio = 6.98;
    public static final double AngleGearRatio = 12.1;
    public static final double SpeedCircumference = 0.1016 * Math.PI;
    public static final double DriverRampRatePeriod = 0.05; // TODO: GET REAL
    public static final double AutonomousRampRatePeriod = 0.05; // TODO: GET REAL
    public static final LinearVelocity MaxFreeSpeed = FeetPerSecond.of(15.01);
    public static final AngularVelocity MaxRotationSpeed = RadiansPerSecond.of(12.0); // TODO: GET REAL
    public static final PIDValue DefaultModuleAnglePIDValue = new PIDValue(60.0, 12.0, 0.0);
    public static final PIDFValue DefaultModuleVelocityPIDFValues =
        new PIDFValue(5.0, 0.0, 0.0, 2.19);

    /** This stores our constants for the front left swerve module. */
    public static class SwerveFrontLeftConstants {
      public static final Translation2d ModOffset = new Translation2d(0.3048, 0.3048);
      public static final InvertedValue InvertedSpeed = InvertedValue.CounterClockwise_Positive;
      public static final Rotation2d AngleEncoderOffset = Rotation2d.fromDegrees(-28.92);
    }

    /** This stores our constants for the front right swerve module. */
    public static class SwerveFrontRightConstants {
      public static final Translation2d ModOffset = new Translation2d(0.3048, -0.3048);
      public static final InvertedValue InvertedSpeed = InvertedValue.Clockwise_Positive;
      public static final Rotation2d AngleEncoderOffset = Rotation2d.fromDegrees(111.71);
    }

    /** This stores our constants for the back left swerve module. */
    public static class SwerveBackLeftConstants {
      public static final Translation2d ModOffset = new Translation2d(-0.3048, 0.3048);
      public static final InvertedValue InvertedSpeed = InvertedValue.CounterClockwise_Positive;
      public static final Rotation2d AngleEncoderOffset = Rotation2d.fromDegrees(129.56);
    }

    /** This stores our constants for the back right swerve module. */
    public static class SwerveBackRightConstants {
      public static final Translation2d ModOffset = new Translation2d(-0.3048, -0.3048);
      public static final InvertedValue InvertedSpeed = InvertedValue.Clockwise_Positive;
      public static final Rotation2d AngleEncoderOffset = Rotation2d.fromDegrees(96.33);
    }
  }

  /** This contains constants for our vision system. */
  public static class VisionConstants {
    private static CameraSpecs initializeLimelight3G() {
      CameraSpecs limeLight3G = new CameraSpecs();
      limeLight3G.minimum_error = 0.02;
      limeLight3G.error_exponent = 2.2;
      limeLight3G.distance_scalar = 1 / 3.15;
      limeLight3G.error_multiplier = 10.0;  // Higher values reduce confidence, tuned to 10 from 1 based on Isaac's feedback.
      limeLight3G.tag_count_scalar = 1.0;
      limeLight3G.VFOV = 56.0;
      limeLight3G.HFOV = 80.0;
      limeLight3G.max_speed_acceptable = 1.0; //mps
      limeLight3G.max_distance_acceptable = 4.0; // meters
      limeLight3G.max_rotation_acceptable = 0.8; //rps
      limeLight3G.confidence_requirement = 0.5; 
      return limeLight3G;
    }

    public static final CameraSpecs limeLight3GSpecs = initializeLimelight3G();
  }

  /** This contains constants for our entire lift system. */
  public static class MidLiftConstants {
    /** Contains values for different known lift poses. */
    public static class LiftPoses {
      public static final LiftPose Stow = new LiftPose("Stow", 90, 0.01, 0.0);
      public static final LiftPose Handoff = new LiftPose("Handoff", 30.0, 0.4, 0.0);
      public static final LiftPose ScoreL1 = new LiftPose("ScoreL1", 29.0, 0.59, -20.0);
      public static final LiftPose ScoreL2 = new LiftPose("ScoreL2", 66.0, 0.575, -100.0);
      public static final LiftPose ScoreL3 = new LiftPose("ScoreL3", 75.5, 0.93, -103.5);
      public static final LiftPose ScoreL4 = new LiftPose("ScoreL4", 81.5, 1.59, -120.0);
      public static final LiftPose AlgaeHigh = new LiftPose("AlgaeHigh", 64.0, 1.0, -72.0);
      public static final LiftPose AlgaeLow = new LiftPose("AlgaeLow", 51.0, 0.71, -60.0);
      public static final LiftPose HpIntake = new LiftPose("HpIntake", 72.0, 0.057, -33.5); // Last updated 2/22/25
      public static final LiftPose ClimbPrep = new LiftPose("ClimbPrep", 80.0, 0.1, 0.0);
      public static final LiftPose Climb = new LiftPose("Climb", 39.0, 0.1, 0.0); // 39 or 38.8 also 47 might work for pivot angle
      public static final LiftPose HoldCoral = new LiftPose("HoldCoral", 90.0, 0.567, 0.0);
      public static final LiftPose BottomBucket = new LiftPose("BottomBucket", 90.0, 0.01, 0);
      public static final LiftPose TopBucket = new LiftPose("TopBucket", 90.0, 0.6, 0);
    }

    /** This contains constants for our Base Pivot. */
    public static class BasePivotConstants {
      public static final Rotation2d MinAngle = Rotation2d.fromDegrees(18); // TODO: go back to 20
      public static final Rotation2d MaxAngle = Rotation2d.fromDegrees(90);

      public static final double kP = 400.0;
      public static final double kI = 10.00;
      public static final double kD = 0.0;
      public static final double kG = 0.4;
      public static final double kS = 0.25;
      public static final double kV = 0.12;
      public static final double kA = 0.01;

      // Motion Magic
      public static final double MMCruiseVelocity = 10;
      public static final double MMAcceleration = 10;
      public static final double MMJerk = 100;

      public static final double SupplyCurrentLimit = 50;
      public static final double StatorCurrentLimit = 50; // TODO: up when climbing

      // Sensor Feedback
      public static final double RotorToSensorRatio = 302.4;
      public static final double SensorToMechanismRatio = 1.0;
      
      // Ramp Rates
      public static final double VoltageClosedLoopRampPeriod = 0.1;
      // Offset
      public static final double canCoderOffsetDegrees = -107.5;
    }

    /** This contains constants for our Gripper Pivot. */
    public static class GripperPivotConstants {

      public static final Rotation2d MinAngle = Rotation2d.fromDegrees(-113); // TODO: [-140, 0] maps to the same CW+ as the base pivot. Are we okay with only negative numbers?
      public static final Rotation2d MaxAngle = Rotation2d.fromDegrees(0);
      public static final Rotation2d SafeAngle = Rotation2d.fromDegrees(0); 
      public static final Rotation2d SafeAngleTolerance = Rotation2d.fromDegrees(4);

      public static final double kP = 30.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kG = 0.13;
      public static final double kS = 0.25;
      public static final double kV = 0.0;
      public static final double kA = 0.00;

      // Motion Magic // TODO: get real values
      public static final double MMCruiseVelocity = 80;
      public static final double MMAcceleration = 160;
      public static final double MMJerk = 1600;

      public static final double SupplyCurrentLimit = 10;
      public static final double StatorCurrentLimit = 10;

      // Sensor Feedback // TODO: get real values
      public static final double RotorToSensorRatio = 51.04;
      public static final double SensorToMechanismRatio = 1;
      
      // Ramp Rates // TODO: get real values
      public static final double VoltageClosedLoopRampPeriod = 0.1;

      //Offset // TODO: Get Real
      public static final double canCoderOffsetDegrees = -74;
    }

    /** This contains constants for our Extender. */
    public static class ExtenderConstants {
      public static final double MinLengthMeter = 0.0;
      public static final double MaxLengthMeter = 1.6;
      public static final double BucketTopClearanceMeter = 0.567;
      public static final double BucketBottomClearanceMeter = 0.013;
      public static final boolean HasMagnetSensor = true; // TODO: enable hasMagneto 

      // Slot 0 Configs
      public static final double kP = 150.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kG = 0.35;
      public static final double kS = 0.5;
      public static final double kV = 0.12;
      public static final double kA = 0.01;

      // Motion Magic
      public static final double MMCruiseVelocity = 3;
      public static final double MMAcceleration = 10;
      public static final double MMJerk = 100;

      // Current limits
      public static final double SupplyCurrentLimit = 50;
      public static final double StatorCurrentLimit = 50;

      // Sensor Feedback
      public static final double RotorToSensorRatio = 1.0;
      public static final double SensorToMechanismRatio = 41.33915313;

      // Ramp Rates
      public static final double VoltageClosedLoopRampPeriod = 0.1;

      
    }

    /** This contains constants for our Gripper. */
    public static class GripperConstants {
      public static final double CoralDropDebounceSeconds = 0.5;
    }
  }

  /** This contains constants for our Intake. */
  public static class IntakeConstants {
    public static final Rotation2d StowAngle = Rotation2d.fromDegrees(100.0);
    public static final Rotation2d DeployAngle = Rotation2d.fromDegrees(10.0);
    public static final Rotation2d HandoffAngle = Rotation2d.fromDegrees(150.0);

    public static final double StowSpeed = 0.0;
    public static final double DeploySpeed = 1.0;
    public static final double HandoffPrepSpeed = 0.0;
    public static final double HandoffSpeed = -1.0;
  }

  /** This contains constants for the field. */
  public static class FieldDimensions {
    // Value taken from Limelight fmap
    public static final double FieldLength = 17.5482504;
    // Value taken from Limelight fmap
    public static final double FieldWidth = 8.0519016;
    // Transform to the Driver Perspective Left Reef from the perspective of the April Tag
    public static final Transform2d ReefBranchLeft =
        new Transform2d(-0.0536, -0.1643, Rotation2d.fromDegrees(0));
    // Transform to the Driver Perspective Right Reef from the perspective of the April Tag
    public static final Transform2d ReefBranchRight =
        new Transform2d(-0.0536, 0.1643, Rotation2d.fromDegrees(0));
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

  /** This contains constants for our robot dimensions. */
  public static class RobotDimensions {
    // Includes the Bumpers
    public static final double FrontBackLengthMeters = 0.9157;
    // Includes the Bumpers
    public static final double SideSideLengthMeters = 0.9157;
    // Buffer space to use between the end effector and an interaction point
    public static final double CoralPlacementMargin = 0.03;
    // Robot length buffer
    public static final double RobotToReefMargin = 0.005; // This is in meters
    // Distance from the robot origin to the axle for the Base Pivot
    public static final Transform2d BasePivotOffset =
        new Transform2d(-0.1973, 0.1762, Rotation2d.fromDegrees(0));

    /** Distance from the dynamic lift position to the wrist on the gripper mechanism.
     * If the lift were all the way down, then this would be the distance from the center of the axle of 
     * the Base Pivot to the center of the axle of the wrist pivot. 
     * This must be rotated by the angle of the Base Pivot at some point in the Forward Kinematics. */
    public static final Transform2d LiftToWristOffset =
        new Transform2d(0.1784, 0.4259, Rotation2d.fromDegrees(0));

    /** Distance from the center of the wrist's axle to the point used for the EndEffector calculations 
     * Presumably this is just beyond the end of the wheels but should be just past where the Coral is. */
    public static final Transform2d WristToEndEffector =
        new Transform2d(0, 0, Rotation2d.fromDegrees(0));
  }
}
