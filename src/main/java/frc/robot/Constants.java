package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.utils.SafetyUtil.GripperPivotSafety;
import java.util.ArrayList;
import java.util.List;

/** This class holds all of our 2025 constants. */
public final class Constants {

  /** This has contants that can be used throughout the robot. */
  public static class GeneralConstants {
    public static final double RobotMassKg = 54.43;
    // public static final Mode RobotMode = Mode.REPLAY;
    public static final Mode RobotMode = Robot.isReal() ? Mode.REAL : Mode.SIM;
    public static final Pose2d InitialRobotPose = new Pose2d(7.5, 4, Rotation2d.fromDegrees(180));
    public static final double RumbleIntensity = 1; //Rumble Intensity is on a range from zero to one.
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
    public static final int GripperCoralMotorCANID = 52; // TODO: set on robot
    public static final int GripperAlgaeMotorCANID = 53; // TODO: set on robot

    // Intake (80s) RIP 2025-2025 for now for now
    public static final int IntakeMotor1CANID = 80; // TODO: set on robot
    public static final int IntakeMotorBCANID = 81; // TODO: set on robot
  }

  /** This contains constants for all our IO ports on the RIO. */
  public static class IoPortsConstants {
    public static final int CoralChannelIDFront = 3;
    public static final int CoralChannelIDBack = 1;
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
    public static final double SpeedCircumference = Inches.of(4).in(Meters) * Math.PI;
    public static final double DriverRampRatePeriod = 0.05; // TODO: GET REAL
    public static final double AutonomousRampRatePeriod = 0.05; // TODO: GET REAL
    public static final double DriverSlowRampRatePeriod = 0.1;
    public static final LinearVelocity MaxFreeSpeed = FeetPerSecond.of(15.01);
    public static final AngularVelocity MaxRotationSpeed = RadiansPerSecond.of(12.0); // TODO: GET REAL
    public static final PIDValue DefaultModuleAnglePIDValue = new PIDValue(60.0, 12.0, 0.0);
    public static final PIDFValue DefaultModuleVelocityPIDFValues =
        new PIDFValue(5.0, 0.0, 0.0, 2.19);
    public static final PIDValue AutoAnglePID = new PIDValue(0.04, 0.0001, 0.0);
    public static final PIDValue AutoTranslationPID = new PIDValue(1.2, 0.06, 0.1);

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
      limeLight3G.error_multiplier = 4.0;  // Higher values reduce confidence, tuned to 10 from 1 based on Isaac's feedback.
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
    // Fudge factor to adjust camera capture time between the limelight and the robot's timestamps
    public static final double timeOffset = 0.00;
  }

  /** This contains constants for our entire arm system. */
  public static class ArmConstants {
    /** Contains values for different known arm poses. */
    public static class ArmPoses {
      public static final ArmPose Stow = new ArmPose("Stow", 80.0, 0.25, -18.8);
      // public static final ArmPose Handoff = new ArmPose("Handoff", 30.0, 0.4, 0.0);
      public static final ArmPose ScoreL1 = new ArmPose("ScoreL1", 75.5, 0.28, -75.9);
      public static final ArmPose ScoreL2 = new ArmPose("ScoreL2", 75.0, 0.56, -80.0);
      public static final ArmPose ScoreL3 = new ArmPose("ScoreL3", 80.0, 0.93, -84.0);
      public static final ArmPose ScoreL4 = new ArmPose("ScoreL4", 75.5, 1.31, 48.0);
      public static final ArmPose AlgaeHigh = new ArmPose("AlgaeHigh", 75.76, 0.85, -70.9);
      public static final ArmPose AlgaeLow = new ArmPose("AlgaeLow", 63.54, 0.55, -54.84);
      public static final ArmPose HpIntake = new ArmPose("HpIntake", 76.6, 0.51, -36.0); // Last updated 2/22/25
      public static final ArmPose ClimbPrep = new ArmPose("ClimbPrep", 90.0, 0.3, -20.0);
      public static final ArmPose Climb = new ArmPose("Climb", 13.0, 0.34, 0.0); // 39 or 38.8 also 47 might work for pivot angle
      public static final ArmPose HoldCoral = new ArmPose("HoldCoral", 75.5, 0.28, -75.9);
      public static final ArmPose HoldAlgae = new ArmPose("HoldAlgae", 83.5, 0.34, -38.3); // TODO tune this
      public static final ArmPose ScoreBarge = new ArmPose("ScoreBarge", 83.5, 1.65, 60); //TODO tune this
      public static final ArmPose ScoreProcessor = new ArmPose("ScoreProcessor", 47.9, 0.0, -40.5); //TODO tune this
      public static final ArmPose FloorIntakeCoral = new ArmPose("FloorIntakeCoral", 15.0, 0.28, -11.5); //TODO tune this
      public static final ArmPose FloorIntakeAlgae = new ArmPose("FloorIntakeAlgae", 44.9, 0.34, -104.8); //TODO tune this
    }

    /** This contains constants for our Base Pivot. */
    public static class BasePivotConstants {
      public static final Rotation2d MinAngle = Rotation2d.fromDegrees(13); // TODO: go back to 20
      public static final Rotation2d MaxAngle = Rotation2d.fromDegrees(90);
      public static final Rotation2d LowerSafetyAngle = Rotation2d.fromDegrees(45);

      public static final double kP = 400.0;
      public static final double kI = 10.00;
      public static final double kD = 0.0;
      public static final double kG = 0.4;
      public static final double kS = 0.25;
      public static final double kV = 0.12;
      public static final double kA = 0.01;

      // Motion Magic
      public static final double MMCruiseVelocity = 0.3;
      public static final double MMAcceleration = 1;
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

    /** This contains constants for our Extender. */
    public static class ExtenderConstants {
      public static final double MinLengthMeter = 0.0;
      public static final double MaxLengthMeter = 1.68;
      
      public static final boolean HasMagnetSensor = true; // TODO: Magneto enable


      // Slot 0 Configs
      public static final double kP = 150.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kG = 0.83;
      public static final double kS = 0.5;
      public static final double kV = 0.12;
      public static final double kA = 0.01;

      // Motion Magic
      public static final double MMCruiseVelocity = 2;
      public static final double MMAcceleration = 5;
      public static final double MMJerk = 100;

      public static final double MMUpCruiseVelocity = 2;
      public static final double MMUpAcceleration = 5;
      public static final double MMUpJerk = 100;

      // Current limits
      public static final double SupplyCurrentLimit = 65;
      public static final double StatorCurrentLimit = 65;

      // Sensor Feedback
      public static final double RotorToSensorRatio = 1.0;
      public static final double SensorToMechanismRatio = 41.33915313;

      // Ramp Rates
      public static final double VoltageClosedLoopRampPeriod = 0.1;
    }

    /** This contains constants for our Gripper Pivot. */
    public static class GripperPivotConstants {

      public static final Angle TrueSafeAngle = Degrees.of(-25);

      public static final GripperPivotSafety low = new GripperPivotSafety(Meters.of(-0.05), Meters.of(0.2), Degrees.of(-50), Degrees.of(-25));
      public static final GripperPivotSafety mid = new GripperPivotSafety(Meters.of(0.2), Meters.of(0.5), Degrees.of(-115), Degrees.of(5));
      public static final GripperPivotSafety high = new GripperPivotSafety(Meters.of(0.5), Meters.of(1.8), Degrees.of(-115), Degrees.of(60));
     
      public static final List<GripperPivotSafety> Safeties = new ArrayList<GripperPivotSafety>() {{
          add(low);
          add(mid);
          add(high);
        }};

      public static final double kP = 30.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kG = 0.43;
      public static final double kS = 0.25;
      public static final double kV = 0.0;
      public static final double kA = 0.00;

      // Motion Magic // TODO: get real values
      public static final double MMCruiseVelocity = 0.2; // 80
      public static final double MMAcceleration = 0.2; // 20
      public static final double MMJerk = 1600;

      public static final double SupplyCurrentLimit = 30;
      public static final double StatorCurrentLimit = 30;

      // Sensor Feedback // TODO: get real values
      public static final double RotorToSensorRatio = 51.04;
      public static final double SensorToMechanismRatio = 1;
      
      // Ramp Rates // TODO: get real values
      public static final double VoltageClosedLoopRampPeriod = 0.1;

      //Offset // TODO: Get Real
      public static final double canCoderOffsetDegrees = -4.0;
    }

    /** This contains constants for our Gripper. */
    public static class GripperConstants {
      public static final double CoralDropDebounceSeconds = 0.5;
      public static final double AlgaeDropDebounceSeconds = 0.5;
      public static final Current AlgaeStatorCurrentLimit = Amps.of(40);
      public static final Current AlgaeSupplyCurrentLimit = Amps.of(40);
      public static final Current CoralStatorCurrentLimit = Amps.of(60);
      public static final Current CoralSupplyCurrentLimit = Amps.of(60);

      // Outake Coral
      public static final double OutakeCoralSpeed = 0.5;
      public static final double OutakeCoralOnAlgaeMotorSpeed = -0.5;
      public static final double OutakeInvertedCoralOnAlgaeMotorSpeed = 0.5;
      public static final double OutakeInvertedCoralSpeed = -0.5;

      // Coral Intake Floor
      public static final double IntakeCoralOnAlgaeMotorSpeed = 1.0;
      public static final double IntakeCoralOnAlgaeSlowMotorSpeed = 0.4;
      public static final double IntakeCoralSpeed = -0.75;
      public static final double IntakeCoralSlow = -0.4;

      // Algae Speeds
      public static final double IntakeAlgaeSpeed = -1.0;
      public static final double HoldAlgaeSpeed = -1.0;
      public static final double OutakeAlgaeSpeed = 0.5;

      // Coral Intake HP
      public static final double HpIntakeCoralSpeed = -0.4;
      public static final double HpIntakeCoralSlowSpeed = -0.2;
      public static final double HpIntakeCoralOnAlgaeMotorSpeed = 0.5;
      public static final double HpIntakeCoralOnAlgaeSlowMotorSpeed = 0.2;
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
    // Trasform the Driver Perspective Center Reef from the perspective of the April Tag
    public static final Transform2d ReefCenterBranch =
        new Transform2d(-0.0536, 0, Rotation2d.fromDegrees(0));
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
    // Distance between center of robot + safety +center of reef
    public static final Distance ReefScoringDistanceThreshold = Meters.of((RobotDimensions.FrontBackLengthMeters / 2) + 0.912493).plus(Inches.of(13)); 
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
    public static final double RobotToReefMargin = 0.015; // This is in meters
    public static final double WristToCoralIntakeAxle = 0.169627; // -0.209097 down, but who cares?
    public static final Angle AlgaeBarAngle = Degrees.of(117.160050);
    // Distance from the robot origin to the axle for the Base Pivot
    public static final Transform2d BasePivotOffset = new Transform2d(-0.1973, 0.1762, Rotation2d.kZero);

    /** Distance from the dynamic arm position to the wrist on the gripper mechanism.
     * If the arm were all the way down, then this would be the distance from the center of the axle of 
     * the Base Pivot to the center of the axle of the wrist pivot. 
     * This must be rotated by the angle of the Base Pivot at some point in the Forward Kinematics. */
    public static final Transform2d ArmToWristOffset = new Transform2d(0.1784, 0.4259, Rotation2d.kZero);

    // Distance from the wrist to the back of the coral, placed as if it was as in the CAD mockup (Mar/14)
    public static final Transform2d WristToCoralFront = new Transform2d(0.382352, -0.164647, Rotation2d.kZero);
    // Distance from the wrist to the front of the coral, placed as if it was as in the CAD mockup (Mar/14)
    public static final Transform2d WristToCoralBack = new Transform2d(0.080727, -0.164647, Rotation2d.kZero);
  }
}
