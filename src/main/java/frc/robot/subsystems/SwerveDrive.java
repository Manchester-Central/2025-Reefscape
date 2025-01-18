// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chaos131.swerve.BaseSwerveDrive;
import com.chaos131.swerve.BaseSwerveModule;
import com.chaos131.swerve.SwerveConfigs;
import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule.AbsoluteEncoderConfig;
import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule.AngleControllerConfig;
import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule.DriveConfig;
import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule.SpeedControllerConfig;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SwerveConstants;
import java.util.function.Supplier;

/** Add your docs here. */
public class SwerveDrive extends BaseSwerveDrive {

  private RobotConfig m_pathPlannerConfig;

  private SwerveDrive(
      BaseSwerveModule[] swerveModules,
      SwerveConfigs swerveConfigs,
      Supplier<Rotation2d> getRotation)
      throws Exception {
    super(swerveModules, swerveConfigs, getRotation);

    try {
      m_pathPlannerConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      throw e;
    }

    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> move(speeds), // Method that will drive the robot given ROBOT
        // RELATIVE ChassisSpeeds. Also optionally
        // outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following
            // controller for holonomic drive trains
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
        m_pathPlannerConfig, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );
  }

  public static SwerveDrive SeparateConstructor(Pigeon2 gyrPigeon2) throws Exception {
    SwerveConfigs swerveConfigs = new SwerveConfigs();
    SwerveModule2025 frontLeftSwerveModule =
        new SwerveModule2025(
            "FL",
            new Translation2d(SwerveConstants.FLModoffsetX, SwerveConstants.FLModoffsetY),
            new SpeedControllerConfig(
                SwerveConstants.FLSpeedCANID,
                SwerveConstants.FLInvertedSpeed,
                SwerveConstants.FLSpeedGearRatio,
                SwerveConstants.FLSpeedCircumference),
            new AngleControllerConfig(
                SwerveConstants.FLAngleCANID,
                SwerveConstants.FLInvertedAngle,
                SwerveConstants.FLAngleGearRatio),
            new AbsoluteEncoderConfig(
                SwerveConstants.FLAbsoEncoCANID,
                SwerveConstants.FLInvertedEncoder,
                SwerveConstants.FLAngleEncoderOffset),
            new DriveConfig(
                SwerveConstants.FLDriverRampRatePeriod,
                SwerveConstants.FLAutonomousRampRatePeriod));
    SwerveModule2025 frontRightSwerveModule =
        new SwerveModule2025(
            "FR",
            new Translation2d(SwerveConstants.FRModoffsetX, SwerveConstants.FRModoffsetY),
            new SpeedControllerConfig(
                SwerveConstants.FRSpeedCANID,
                SwerveConstants.FRInvertedSpeed,
                SwerveConstants.FRSpeedGearRatio,
                SwerveConstants.FRSpeedCircumference),
            new AngleControllerConfig(
                SwerveConstants.FRAngleCANID,
                SwerveConstants.FRInvertedAngle,
                SwerveConstants.FRAngleGearRatio),
            new AbsoluteEncoderConfig(
                SwerveConstants.FRAbsoEncoCANID,
                SwerveConstants.FRInvertedEncoder,
                SwerveConstants.FRAngleEncoderOffset),
            new DriveConfig(
                SwerveConstants.FRDriverRampRatePeriod,
                SwerveConstants.FRAutonomousRampRatePeriod));
    SwerveModule2025 backLeftSwerveModule =
        new SwerveModule2025(
            "BL",
            new Translation2d(SwerveConstants.BLModoffsetX, SwerveConstants.BLModoffsetY),
            new SpeedControllerConfig(
                SwerveConstants.BLSpeedCANID,
                SwerveConstants.BLInvertedSpeed,
                SwerveConstants.BLSpeedGearRatio,
                SwerveConstants.BLSpeedCircumference),
            new AngleControllerConfig(
                SwerveConstants.BLAngleCANID,
                SwerveConstants.BLInvertedAngle,
                SwerveConstants.BLAngleGearRatio),
            new AbsoluteEncoderConfig(
                SwerveConstants.BLAbsoEncoCANID,
                SwerveConstants.BLInvertedEncoder,
                SwerveConstants.BLAngleEncoderOffset),
            new DriveConfig(
                SwerveConstants.BLDriverRampRatePeriod,
                SwerveConstants.BLAutonomousRampRatePeriod));
    SwerveModule2025 backRightSwerveModule =
        new SwerveModule2025(
            "BR",
            new Translation2d(SwerveConstants.BRModoffsetX, SwerveConstants.BRModoffsetY),
            new SpeedControllerConfig(
                SwerveConstants.BRSpeedCANID,
                SwerveConstants.BRInvertedSpeed,
                SwerveConstants.BRSpeedGearRatio,
                SwerveConstants.BRSpeedCircumference),
            new AngleControllerConfig(
                SwerveConstants.BRAngleCANID,
                SwerveConstants.BRInvertedAngle,
                SwerveConstants.BRAngleGearRatio),
            new AbsoluteEncoderConfig(
                SwerveConstants.BRAbsoEncoCANID,
                SwerveConstants.BRInvertedEncoder,
                SwerveConstants.BRAngleEncoderOffset),
            new DriveConfig(
                SwerveConstants.BRDriverRampRatePeriod,
                SwerveConstants.BRAutonomousRampRatePeriod));
    SwerveModule2025[] swerveModule2025s = {
      frontLeftSwerveModule, frontRightSwerveModule, backLeftSwerveModule, backRightSwerveModule
    };
    SwerveDrive swerveDrive =
        new SwerveDrive(swerveModule2025s, swerveConfigs, () -> gyrPigeon2.getRotation2d());
    return swerveDrive;
  }

  public void pathPlannerRobotRelative(
      ChassisSpeeds chassisSpeeds, DriveFeedforwards driveFeedforwards) {
    pathPlannerRobotRelative(chassisSpeeds);
    // Investigate decision regarding the use of feed forwards

  }

  public Command followPathCommand(String pathName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

      return new FollowPathCommand(
          path,
          this::getPose, // Robot pose supplier
          this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          this::pathPlannerRobotRelative, // Method that will drive the robot given ROBOT RELATIVE
          // ChassisSpeeds, AND feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path
              // following controller for holonomic drive
              // trains
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
              ),
          m_pathPlannerConfig, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the
            // red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this // Reference to this subsystem to set requirements
          );
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }
}
