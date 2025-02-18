// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chaos131.robot.ChaosRobot.Mode;
import com.chaos131.swerve.BaseSwerveDrive;
import com.chaos131.swerve.SwerveConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.SwerveBackLeftConstants;
import frc.robot.Constants.SwerveConstants.SwerveBackRightConstants;
import frc.robot.Constants.SwerveConstants.SwerveFrontLeftConstants;
import frc.robot.Constants.SwerveConstants.SwerveFrontRightConstants;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class SwerveDrive extends BaseSwerveDrive {

  private RobotConfig m_pathPlannerConfig;
  private DriveTrainSimulationConfig m_simDriveTrain;
  private SwerveDriveSimulation m_driveSim;
  private SelfControlledSwerveDriveSimulation m_simulatedDrive;

  private SwerveDrive(
      SwerveModule2025[] swerveModules,
      SwerveConfigs swerveConfigs,
      Supplier<Rotation2d> getRotation)
      throws Exception {
    super(swerveModules, swerveConfigs, getRotation);

    resetPose(GeneralConstants.InitialRobotPose);
    m_simDriveTrain = DriveTrainSimulationConfig.Default();
    m_driveSim = new SwerveDriveSimulation(m_simDriveTrain, GeneralConstants.InitialRobotPose);
    // Creating the SelfControlledSwerveDriveSimulation instance
    m_simulatedDrive = new SelfControlledSwerveDriveSimulation(m_driveSim);
    // Register the drivetrain simulation to the simulation world
    if (GeneralConstants.RobotMode == Mode.SIM) {
      SimulatedArena.getInstance()
          .addDriveTrainSimulation(m_simulatedDrive.getDriveTrainSimulation());
    }

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
        new PPHolonomicDriveController(// PPHolonomicController is the built in path following
            // controller for holonomic drive trains
            new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(2.0, 0.0, 0.0) // Rotation PID constants
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

  /**
   * Creates a new swerve drive with all the values for 2025's robot.
   */
  public static SwerveDrive createSwerveDrive(Pigeon2 gyrPigeon2) throws Exception {
    SwerveConfigs swerveConfigs =
        new SwerveConfigs()
            .setMaxRobotSpeed_mps(SwerveConstants.MaxFreeSpeedMPS)
            .setMaxRobotRotation_radps(SwerveConstants.MaxRotationSpeedRadPS)
            .setDefaultModuleVelocityPIDFValues(SwerveConstants.DefaultModuleVelocityPIDFValues)
            .setDefaultModuleAnglePIDValues(SwerveConstants.DefaultModuleAnglePIDValue)
            .setDebugMode(true);
    SwerveModule2025 frontLeftSwerveModule =
        new SwerveModule2025(
            "FL",
            SwerveFrontLeftConstants.ModOffset,
            CanIdentifiers.FLSpeedCANID,
            CanIdentifiers.FLAngleCANID,
            CanIdentifiers.FLAbsoEncoCANID,
            SwerveFrontLeftConstants.InvertedSpeed,
            SwerveFrontLeftConstants.AngleEncoderOffset);
    SwerveModule2025 frontRightSwerveModule =
        new SwerveModule2025(
            "FR",
            SwerveFrontRightConstants.ModOffset,
            CanIdentifiers.FRSpeedCANID,
            CanIdentifiers.FRAngleCANID,
            CanIdentifiers.FRAbsoEncoCANID,
            SwerveFrontRightConstants.InvertedSpeed,
            SwerveFrontRightConstants.AngleEncoderOffset);
    SwerveModule2025 backLeftSwerveModule =
        new SwerveModule2025(
            "BL",
            SwerveBackLeftConstants.ModOffset,
            CanIdentifiers.BLSpeedCANID,
            CanIdentifiers.BLAngleCANID,
            CanIdentifiers.BLAbsoEncoCANID,
            SwerveBackLeftConstants.InvertedSpeed,
            SwerveBackLeftConstants.AngleEncoderOffset);
    SwerveModule2025 backRightSwerveModule =
        new SwerveModule2025(
            "BR",
            SwerveBackRightConstants.ModOffset,
            CanIdentifiers.BRSpeedCANID,
            CanIdentifiers.BRAngleCANID,
            CanIdentifiers.BRAbsoEncoCANID,
            SwerveBackRightConstants.InvertedSpeed,
            SwerveBackRightConstants.AngleEncoderOffset);
    SwerveModule2025[] swerveModule2025s = {
      frontLeftSwerveModule, frontRightSwerveModule, backLeftSwerveModule, backRightSwerveModule
    };
    SwerveDrive swerveDrive =
        new SwerveDrive(swerveModule2025s, swerveConfigs, () -> gyrPigeon2.getRotation2d());
    return swerveDrive;
  }

  @Override
  public void move(ChassisSpeeds chassisSpeeds) {
    chassisSpeeds.vxMetersPerSecond =
        MathUtil.clamp(chassisSpeeds.vxMetersPerSecond, -1, 1)
            * m_swerveConfigs.maxRobotSpeed_mps()
            * TranslationSpeedModifier;
    chassisSpeeds.vyMetersPerSecond =
        MathUtil.clamp(chassisSpeeds.vyMetersPerSecond, -1, 1)
            * m_swerveConfigs.maxRobotSpeed_mps()
            * TranslationSpeedModifier;
    chassisSpeeds.omegaRadiansPerSecond =
        MathUtil.clamp(chassisSpeeds.omegaRadiansPerSecond, -1, 1)
            * m_swerveConfigs.maxRobotRotation_radps()
            * RotationSpeedModifier;
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, m_swerveConfigs.maxRobotSpeed_mps());
    if (GeneralConstants.RobotMode == Mode.SIM) {
      m_simulatedDrive.runSwerveStates(states);
    }
    Logger.recordOutput("Swerve/TargetSpeeds", states);
    if (chassisSpeeds.vxMetersPerSecond == 0
        && chassisSpeeds.vyMetersPerSecond == 0
        && chassisSpeeds.omegaRadiansPerSecond == 0) {
      stop();
      return;
    }

    for (var i = 0; i < states.length; i++) {
      m_swerveModules.get(i).setTarget(states[i]);
    }
  }

  /**
   * Allows PathPlanner to drive our robot.
   */
  private void pathPlannerRobotRelative(
      ChassisSpeeds chassisSpeeds, DriveFeedforwards driveFeedforwards) {
    pathPlannerRobotRelative(chassisSpeeds);
    // TODO: Investigate decision regarding the use of feed forwards
  }

  /**
   * Tells the robot to follow a PathPlanner plan.
   */
  public Command followPathCommand(String pathName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

      return new FollowPathCommand(
          path,
          this::getPose, // Robot pose supplier
          this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          this::pathPlannerRobotRelative, // Method that will drive the robot given ROBOT RELATIVE
          // ChassisSpeeds, AND feedforwards
          new PPHolonomicDriveController(// PPHolonomicController is the built in path
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

  @Override
  public Rotation2d getGyroRotation() {
    if (GeneralConstants.RobotMode == Mode.SIM) {
      return m_simulatedDrive != null
          ? m_simulatedDrive.getActualPoseInSimulationWorld().getRotation()
          : GeneralConstants.InitialRobotPose.getRotation();
    }
    return super.getGyroRotation();
  }

  @Override
  public void periodic() {
    if (GeneralConstants.RobotMode == Mode.SIM) {
      m_simulatedDrive.periodic();
      Logger.recordOutput("Swerve/SimPose", m_simulatedDrive.getActualPoseInSimulationWorld());
    }
    super.periodic();
    Logger.recordOutput("Swerve/Pose", getPose());
    Logger.recordOutput("Swerve/CurrentSpeeds", getModuleStates());
  }
}
