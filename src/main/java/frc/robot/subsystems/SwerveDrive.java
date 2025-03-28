// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.chaos131.robot.ChaosRobot.Mode;
import com.chaos131.swerve.BaseSwerveDrive;
import com.chaos131.swerve.SwerveConfigs;
import com.chaos131.util.DashboardNumber;
import com.chaos131.vision.VisionData;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.SwerveBackLeftConstants;
import frc.robot.Constants.SwerveConstants.SwerveBackRightConstants;
import frc.robot.Constants.SwerveConstants.SwerveFrontLeftConstants;
import frc.robot.Constants.SwerveConstants.SwerveFrontRightConstants;
import frc.robot.Robot;
import java.util.Optional;
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
  private PPHolonomicDriveController m_holonomicDriveController = new PPHolonomicDriveController(
      new PIDConstants(1.0, 0.0, 0.0),
      new PIDConstants(2.0, 0.0, 0.0));
  private TimeInterpolatableBuffer<Pose2d> m_pastPoses;
  private CircularBuffer<Rotation2d> m_swerveAngles;
  private DashboardNumber m_minTranslationSpeed = new DashboardNumber("MinTranslationSpeed", 0.2, true, newValue -> {});

  /** A value to help determine if we should use PathPlanner's reset odometry or not. */
  private boolean m_hasReceivedVisionUpdates = false;

  private SwerveDrive(
      SwerveModule2025[] swerveModules,
      SwerveConfigs swerveConfigs,
      Supplier<Rotation2d> getRotation)
      throws Exception {
    super(swerveModules, swerveConfigs, getRotation);

    m_acceptVisionUpdates = SwerveConstants.AcceptVisionUpdates;
    m_pastPoses = TimeInterpolatableBuffer.createBuffer(1);
    m_swerveAngles = new CircularBuffer<>(50);
    m_odometry =
        new SwerveDrivePoseEstimator(
            m_kinematics, getGyroRotation(), getModulePositions(), GeneralConstants.InitialRobotPose,
             VecBuilder.fill(0.1, 0.1, 0.1),
        VecBuilder.fill(2.5, 2.5, 2.5));

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
        this::resetPosePathPlanner, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> move(speeds), // Method that will drive the robot given ROBOT
        // RELATIVE ChassisSpeeds. Also optionally
        // outputs individual module feedforwards
       
        m_holonomicDriveController,
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
            .setMaxRobotSpeed(SwerveConstants.MaxFreeSpeed)
            .setMaxRobotRotation(SwerveConstants.MaxRotationSpeed)
            .setDefaultModuleVelocityPIDFValues(SwerveConstants.DefaultModuleVelocityPIDFValues)
            .setDefaultModuleAnglePIDValues(SwerveConstants.DefaultModuleAnglePIDValue)
            .setDefaultRotationPIDValues(SwerveConstants.AutoAnglePID)
            .setDefaultTranslationPIDValues(SwerveConstants.AutoTranslationPID)
            .setDebugMode(true)
            .setDefaultDriveToTargetTolerance(SwerveConstants.DriveToTargetTolerance);
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

  /**
   * Resets the robot's position on the field - will skip if certain conditions are met.
   *
   *
   * @param targetPose the pose to set to
   */
  public void resetPosePathPlanner(Pose2d targetPose) {
    if (Robot.isReal() && m_hasReceivedVisionUpdates) {
      // If this is a real robot and we have received camera updates, trust the camera updates over PathPlanner's starting pose
      return;
    }
    // If it's a sim robot or we have not received vision updates, we can trust PathPlanner's pose
    resetPose(targetPose);
  }

  /**
  * .
  */

  public boolean atTargetDynamic() {
    if (atTarget() && m_swerveAngles.size() >= 2) {
      Rotation2d angleToTarget2dLast = m_swerveAngles.get(m_swerveAngles.size() - 2);
      Rotation2d angleToTargetLast = m_swerveAngles.getLast();
      Rotation2d dif = angleToTarget2dLast.minus(angleToTargetLast);
      Logger.recordOutput("thresh", SwerveConstants.AtTargetAngleThreshold);
      Logger.recordOutput("dif", dif.getDegrees());
      if (dif.getMeasure().abs(Degrees) > SwerveConstants.AtTargetAngleThreshold.in(Degrees)) {
        return true;
      }
    }
    return false;
  }

  /**
   * To clear the CircularBuffer of swerve rotation values.
   */
  public void clearSwerveAngleBuffer() {
    m_swerveAngles.clear();
  }

  @Override
  public void move(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, m_swerveConfigs.maxRobotSpeed());
    if (GeneralConstants.RobotMode == Mode.SIM) {
      m_simulatedDrive.runSwerveStates(states);
    }
    Logger.recordOutput("Swerve/TargetSpeeds", states);
    // if (chassisSpeeds.vxMetersPerSecond == 0
    //     && chassisSpeeds.vyMetersPerSecond == 0
    //     && chassisSpeeds.omegaRadiansPerSecond == 0) {
    //   stop();
    //   return;
    // }

    for (var i = 0; i < states.length; i++) {
      m_swerveModules.get(i).setTarget(states[i]);
    }
  }

  /**
   * Thread Safe mechanism for adding a vision update from the camera to the swerve estimator.
   * Checks if the SwervePoseEstimator is going to process vision updates at all first.
   */
  public void addVisionMeasurement(VisionData data) {
    if (!m_acceptVisionUpdates) {
      return;
    }
    synchronized (m_odometry) {
      m_hasReceivedVisionUpdates = true;
      m_odometry.addVisionMeasurement(
          data.getPose2d(), data.getTimestampSeconds(), data.getDeviationMatrix());
    }
  }

  /**
   * To change the ramp rate period on the fly.
   *
   * @param newRate time in seconds (clamped to [0-1])
   */
  public void setRampRatePeriod(double newRate) {
    forAllModules((module) -> ((SwerveModule2025) module).setRampRatePeriod(newRate));
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
          // 
          m_holonomicDriveController,
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
    Logger.recordOutput("Swerve/Speed", getRobotSpeed());
    Logger.recordOutput("Swerve/RotationSpeed", getRobotRotationSpeed());
    m_pastPoses.addSample(Timer.getFPGATimestamp(), getPose());
    Translation2d targetPosition = new Translation2d(m_XPid.getSetpoint(), m_YPid.getSetpoint());
    Rotation2d currentAngle = getPose().getTranslation().minus(targetPosition).getAngle();
    Logger.recordOutput("at target dynamic", atTargetDynamic());
    Logger.recordOutput("Command", getCurrentCommand() != null ? getCurrentCommand().getName() : "");
    m_swerveAngles.addLast(currentAngle);
  }

  /**
   * Get past pose.
   *
   * @param timeStamp in seconds
   * @return pose
   */
  public Optional<Pose2d> getPastPose(double timeStamp) {
    return m_pastPoses.getSample(timeStamp);
  }

  @Override
  public void moveToTarget(double maxTranslationSpeedPercent) {
    Pose2d pose = getPose();

    Translation2d difference =
        pose.getTranslation().minus(new Translation2d(m_XPid.getSetpoint(), m_YPid.getSetpoint()));

    var normalizedDifference = difference.div(difference.getNorm());
    maxTranslationSpeedPercent *= m_swerveConfigs.maxRobotSpeed().in(MetersPerSecond); //TODO: do this the right way in shared code

    double x =
        MathUtil.clamp(
            m_XPid.calculate(pose.getX()),
            -(maxTranslationSpeedPercent * normalizedDifference.getX()),
            (maxTranslationSpeedPercent * normalizedDifference.getX()));
    double y =
        MathUtil.clamp(
            m_YPid.calculate(pose.getY()),
            -(maxTranslationSpeedPercent * normalizedDifference.getY()),
            (maxTranslationSpeedPercent * normalizedDifference.getY()));
    double angle = m_AngleDegreesPid.calculate(pose.getRotation().getDegrees());
    moveFieldRelativeForPID(
        Units.MetersPerSecond.of(x), Units.MetersPerSecond.of(y), Units.RadiansPerSecond.of(angle));
  }
 
  /**
   * .
   */
  public void moveToTargetV2(double maxTranslationSpeedPercent) {
    Pose2d pose = getPose();
    double minTranslationSpeed = m_minTranslationSpeed.get();

    Translation2d difference =
        pose.getTranslation().minus(new Translation2d(m_XPid.getSetpoint(), m_YPid.getSetpoint()));

    var normalizedDifference = difference.div(difference.getNorm());
    maxTranslationSpeedPercent *= m_swerveConfigs.maxRobotSpeed().in(MetersPerSecond); //TODO: do this the right way in shared code

    double x =
        MathUtil.clamp(
            m_XPid.calculate(pose.getX()),
            -(maxTranslationSpeedPercent * normalizedDifference.getX()),
            (maxTranslationSpeedPercent * normalizedDifference.getX()));
    x = m_XPid.atSetpoint() ? 0.0 : Math.max(Math.abs(x), minTranslationSpeed) * (x < 0 ? -1 : 1); 
    double y =
        MathUtil.clamp(
            m_YPid.calculate(pose.getY()),
            -(maxTranslationSpeedPercent * normalizedDifference.getY()),
            (maxTranslationSpeedPercent * normalizedDifference.getY()));
    y = m_YPid.atSetpoint() ? 0.0 : Math.max(Math.abs(y), minTranslationSpeed) * (y < 0 ? -1 : 1); 
    double angle = m_AngleDegreesPid.calculate(pose.getRotation().getDegrees());
    moveFieldRelativeForPID(
        Units.MetersPerSecond.of(x), Units.MetersPerSecond.of(y), Units.RadiansPerSecond.of(angle));
  }
}
