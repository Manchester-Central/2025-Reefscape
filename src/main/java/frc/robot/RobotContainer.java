// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.robot.ChaosRobotContainer;
import com.chaos131.util.DashboardNumber;
import com.chaos131.vision.LimelightCamera.LimelightVersion;
import com.chaos131.vision.VisionData;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants.ArmPoses;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.ChangeState;
import frc.robot.commands.DriverRelativeDrive;
import frc.robot.commands.DriverRelativeSetAngleDrive;
import frc.robot.commands.ReefAlignment;
import frc.robot.commands.UpdateHeading;
import frc.robot.commands.WaitForCoral;
import frc.robot.commands.WaitForState;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.MechManager2D;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmState;
import frc.robot.subsystems.arm.Gripper;
import frc.robot.subsystems.arm.SelectedAlgaeState;
import frc.robot.subsystems.arm.SelectedCoralState;
import frc.robot.utils.DriveDirection;
import frc.robot.utils.FieldPoint;
import frc.robot.utils.PathUtil;
import java.util.Map;
import java.util.Random;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer extends ChaosRobotContainer<SwerveDrive> {

  Pigeon2 m_gyro;

  public static Arm m_arm;
  public static Gripper m_gripper;
  public static Camera m_rightCamera;
  public static Camera m_leftCamera;
  public static MechManager2D m_mech2dManager;
  public static SwerveDriveSimulation m_driveSim;
  private Map<String, ArmState> m_aprilTagToAlgaeHeightMap = Map.of(
      "17 ReefTag", ArmState.ALGAE_LOW,
      "18 ReefTag", ArmState.ALGAE_HIGH,
      "19 ReefTag", ArmState.ALGAE_LOW,
      "20 ReefTag", ArmState.ALGAE_HIGH,
      "21 ReefTag", ArmState.ALGAE_LOW,
      "22 ReefTag", ArmState.ALGAE_HIGH
  );

  private Random m_rng = new Random();
  private Trigger m_isAlgaeMode;
  private Trigger m_isCoralMode;
  private Orchestra m_orchestra = new Orchestra();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() throws Exception {
    super();
    m_gyro = new Pigeon2(CanIdentifiers.GyroCANID, CanIdentifiers.CTRECANBus);
    m_swerveDrive = SwerveDrive.createSwerveDrive(m_gyro);
    m_arm = new Arm(m_operator, m_driver);
    m_mech2dManager = new MechManager2D(m_arm);
    m_rightCamera =
        new Camera(
            "limelight-right",
            LimelightVersion.LL3G,
            VisionConstants.limeLight3GSpecs,
            () -> m_swerveDrive.getPose(),
            (data) -> updatePoseEstimator(data),
            () -> m_swerveDrive.getRobotSpeed().in(MetersPerSecond),
            () -> m_swerveDrive.getRobotRotationSpeed().in(RadiansPerSecond));
    m_leftCamera =
        new Camera(
            "limelight-left",
            LimelightVersion.LL3G,
            VisionConstants.limeLight3GSpecs,
            () -> m_swerveDrive.getPose(),
            (data) -> updatePoseEstimator(data),
            () -> m_swerveDrive.getRobotSpeed().in(MetersPerSecond),
            () -> m_swerveDrive.getRobotRotationSpeed().in(RadiansPerSecond));
    NamedCommands.registerCommand("AimReef", PathUtil.driveToClosestPointAutoCommand(FieldPoint.getReefDrivePoses(), m_swerveDrive, 1));
    NamedCommands.registerCommand("AimReefPrep", PathUtil.driveToClosestPointAutoCommand(FieldPoint.getReefDrivePoses(), m_swerveDrive, 1)
        .alongWith(new ChangeState().setArm(ArmState.PREP_L4)));
    NamedCommands.registerCommand("GoToReef8L", new ReefAlignment(FieldPoint.ReefPose8, true, m_swerveDrive));
    NamedCommands.registerCommand("ScoreL1",  new ChangeState().setArm(ArmState.SCORE_L1).andThen(new WaitForState().forArmState(ArmState.STOW)));
    NamedCommands.registerCommand("ScoreL2",  new ChangeState().setArm(ArmState.SCORE_L2).andThen(new WaitForState().forArmState(ArmState.STOW)));
    NamedCommands.registerCommand("ScoreL3",  new ChangeState().setArm(ArmState.SCORE_L3).andThen(new WaitForState().forArmState(ArmState.STOW)));
    NamedCommands.registerCommand("ScoreL4",  new ChangeState().setArm(ArmState.SCORE_L4).andThen(new WaitForState().forArmState(ArmState.STOW)));
    NamedCommands.registerCommand("IntakeFromHP", new ChangeState().setArm(ArmState.INTAKE_FROM_HP).andThen(new WaitForCoral(m_arm)));
    NamedCommands.registerCommand("IntakeFromFloor", new ChangeState().setArm(ArmState.INTAKE_CORAL_FROM_FLOOR).andThen(new WaitForCoral(m_arm)));
    //JOHN SAVE US PLEASE: change hold coral to hold algae
    NamedCommands.registerCommand("AimHP", (PathUtil.driveToClosestPointAutoCommand(FieldPoint.getHpDrivePoses(), m_swerveDrive, 0.5)
        .andThen(new RunCommand(() -> m_swerveDrive.moveRobotRelative(MetersPerSecond.of(1.75), MetersPerSecond.of(0.0), DegreesPerSecond.of(0)), m_swerveDrive)))
        .withDeadline(new ChangeState().setArm(ArmState.INTAKE_FROM_HP).andThen(new WaitForCoral(m_arm))));
    NamedCommands.registerCommand("AimHPOld", PathUtil.driveToClosestPointAutoCommand(FieldPoint.getHpDrivePoses(), m_swerveDrive, 2)
        .withDeadline(new ChangeState().setArm(ArmState.INTAKE_FROM_HP).andThen(new WaitForState().forArmState(ArmState.HOLD_CORAL))));
    NamedCommands.registerCommand("XMode", new RunCommand(() -> m_swerveDrive.setXMode()).withTimeout(0.1));
    buildPathplannerAutoChooser();

    System.out.println(ArmPoses.ScoreL4); // needed for static poses TODO find better way
    
    // Configure the trigger bindings
    configureBindings();
    //m_Orchestra.addInstrument(m_idLift.getPivotMotor());
  }

  /**
   * .
   */
  public void playMusic() {
    if (!m_orchestra.isPlaying()) {

      m_orchestra.loadMusic("chaos2024.chrp");
      m_orchestra.play();
    }
  }

  /**
   * .
   */
  public void stopMusic() {
    if (m_orchestra.isPlaying()) {
      m_orchestra.stop();
    }
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_isAlgaeMode = m_operator.leftBumper().or(m_operator.leftTrigger());
    m_isCoralMode = m_isAlgaeMode.negate();

    m_swerveDrive.setDefaultCommand(new DriverRelativeDrive(m_driver, m_swerveDrive)); 

    // Everything after this is for competition
    m_driver.a().whileTrue(new DriverRelativeSetAngleDrive(m_driver, m_swerveDrive,  () -> {
      FieldPoint pose = FieldPoint.getNearestPoint(m_swerveDrive.getPose(), FieldPoint.getHpDrivePoses());
      return pose.getCurrentAlliancePose().getRotation();
    }, 1.0).alongWith(new ChangeState().setArm(ArmState.INTAKE_FROM_HP).withArmInterrupt(ArmState.STOW)));
    // m_driver.a().whileTrue(PathUtil.driveToClosestPointCommand(FieldPoint.getHpDrivePoses(), m_swerveDrive)
    //     .alongWith(new ChangeState().setArm(ArmState.INTAKE_FROM_HP)
    //     .withArmInterrupt(ArmState.STOW)));
    // m_driver.b().whileTrue(
    //   new DeferredCommand(() -> PathUtil.driveToPoseCommand(new FieldPoint("ClosestBargePoint",
    //     PathUtil.findClosestPointOnLine(m_swerveDrive, FieldPoint.CenterBarge, false)), m_swerveDrive), Set.of(m_swerveDrive))
    //     .andThen(new DriverRelativeSetAngleAndAxisDrive(m_driver, m_swerveDrive, () -> DriveDirection.Towards.getAllianceAngle(), 1.0))
    //     .alongWith(new ChangeState().setArm(ArmState.PREP_BARGE)));
    // m_driver.b().whileTrue(new AlignReefTag(m_swerveDrive, m_leftCamera, m_rightCamera));
    m_driver.b().whileTrue(PathUtil.driveToClosestPointTeleopCommand(FieldPoint.getReefDrivePoses(), m_swerveDrive));
    m_driver.x().whileTrue(new DriverRelativeSetAngleDrive(m_driver, m_swerveDrive, () -> DriveDirection.Away.getAllianceAngle(), 1.0));
    m_driver.y().whileTrue(new ConditionalCommand(
        aimAndPrepCoral(),
        aimAndPrepAlgaeGrab(),
        m_arm.m_gripper::hasCoral));

    m_driver.povUp().onTrue(new UpdateHeading(m_swerveDrive, DriveDirection.Away)); // 0 degrees for blue
    m_driver.povDown().onTrue(new UpdateHeading(m_swerveDrive, DriveDirection.Towards)); // 180 degrees for blue
    m_driver.povLeft().onTrue(new UpdateHeading(m_swerveDrive, DriveDirection.Left)); // 90 degrees for blue
    m_driver.povRight().onTrue(new UpdateHeading(m_swerveDrive, DriveDirection.Right)); // -90 degrees for blue

    //m_driver.start().whileTrue(new ChangeState().setArm(ArmState.POST_CLIMB));
    //m_driver.back().whileTrue(new ChangeState().setArm(ArmState.PREP_CLIMB));

    m_driver.rightBumper().or(m_driver.rightTrigger()).or(m_driver.leftTrigger()).whileTrue(
      new StartEndCommand(() -> m_swerveDrive.setRampRatePeriod(SwerveConstants.DriverSlowRampRatePeriod),
                          () -> m_swerveDrive.setRampRatePeriod(SwerveConstants.DriverRampRatePeriod)));
    m_driver.rightBumper().and(m_isCoralMode).whileTrue(new ChangeState().setArm(() -> m_arm.getSelectedCoralState().PrepState).withArmInterrupt(ArmState.HOLD_CORAL));
    m_driver.rightBumper().and(m_isAlgaeMode).whileTrue(new ChangeState().setArm(() -> m_arm.getSelectedAlgaeState().State).withArmInterrupt(ArmState.HOLD_ALGAE));
    // Right trigger just causes scoring from the prep states

    m_driver.leftTrigger().and(m_isCoralMode).whileTrue(new ChangeState().setArm(ArmState.INTAKE_CORAL_FROM_FLOOR).withArmInterrupt(ArmState.STOW));
    m_driver.leftTrigger().and(m_isAlgaeMode).whileTrue(new ChangeState().setArm(ArmState.INTAKE_ALGAE_FROM_FLOOR).withArmInterrupt(ArmState.STOW));
    m_driver.leftBumper().whileTrue(new ChangeState().setArm(() -> {
      var closestTag = FieldPoint.getNearestPoint(m_swerveDrive.getPose(), FieldPoint.getReefAprilTagPoses());
      return m_aprilTagToAlgaeHeightMap.get(closestTag.getName());
    }).withArmInterrupt(ArmState.STOW));

    m_operator.a().onTrue(new InstantCommand(() -> m_arm.setSelectedCoralState(SelectedCoralState.L1)));
    m_operator.x().onTrue(new InstantCommand(() -> m_arm.setSelectedCoralState(SelectedCoralState.L2)));
    m_operator.b().onTrue(new InstantCommand(() -> m_arm.setSelectedCoralState(SelectedCoralState.L3)));
    m_operator.y().onTrue(new InstantCommand(() -> m_arm.setSelectedCoralState(SelectedCoralState.L4)));

    m_operator.leftBumper().onTrue(new InstantCommand(() -> m_arm.setSelectedAlgaeState(SelectedAlgaeState.PROCESSOR)));
    m_operator.leftTrigger().onTrue(new InstantCommand(() -> m_arm.setSelectedAlgaeState(SelectedAlgaeState.BARGE)));

    m_operator.povUp().whileTrue(new ChangeState().setArm(ArmState.PREP_CLIMB));
    m_operator.povUp().whileTrue(new ChangeState().setArm(ArmState.POST_CLIMB));
    
    m_operator.start().onTrue(new ChangeState().setArm(ArmState.STOW));
    m_operator.back().whileTrue(new ChangeState().setArm(ArmState.MANUAL));

    // Everything after this is for demos and testing

    // v on keyboard 0
    m_simKeyboard.y().onTrue(
      new InstantCommand(() -> {
        System.out.println("Adding Game Piece");
        Logger.recordOutput("DebugPose", FieldPoint.rightSource.getCurrentAlliancePose());
        var newPiece = new ReefscapeCoralOnFly(
            FieldPoint.rightSource.getCurrentAlliancePose().getTranslation(),
            new Translation2d(0.1, 0),
            new ChassisSpeeds(),
            FieldPoint.rightSource.getCurrentAlliancePose().getRotation(), // .plus(Rotation2d.fromDegrees(m_rng.nextDouble(0, 20) - 10)),
            Meters.of(1.5),
            MetersPerSecond.of(m_rng.nextDouble(0.1, 3)),
            Degrees.of(m_rng.nextDouble(0, 10) - 15));
        //newPiece.addGamePieceAfterTouchGround(SimulatedArena.getInstance());
        SimulatedArena.getInstance().addGamePieceProjectile(newPiece);
      })
    );
    // z on keyboard 0
    m_simKeyboard.a().onTrue(new InstantCommand(() -> Gripper.hasCoralGrippedSim = !Gripper.hasCoralGrippedSim));
  }

  /**
   * aim and prep the coral.
   */
  public Command aimAndPrepCoral() {

    return PathUtil.driveToClosestPointTeleopCommandV2(FieldPoint.getReefDrivePoses(), m_swerveDrive);
    // .alongWith(
    //   new WaitUntilCommand(() -> FieldPoint.ReefCenter.getDistance(m_swerveDrive.getPose()).lte(FieldDimensions.ReefScoringDistanceThreshold))
    //   .andThen(
    //     new ChangeState().setArm(() -> m_selectedCoralState.PrepState).withArmInterrupt(ArmState.HOLD_CORAL)
    //   ));
  }

  /**
   * aim algae.
   */
  public Command aimAndPrepAlgaeGrab() {
    return PathUtil.driveToClosestPointTeleopCommand(FieldPoint.getReefCenterDrivePose(), m_swerveDrive);
  }

  @Override
  public void configureDriverController() {
    m_driver = new Gamepad(OperatorConstants.DriverControllerPort);
  }

  @Override
  public void configureOperatorController() {
    m_operator = new Gamepad(OperatorConstants.OperatorControllerPort);
  }

  @Override
  public void configureTesterController() {}

  @Override
  public void configureSimKeyboard() {
    m_simKeyboard = new Gamepad(OperatorConstants.SimulationControllerPort);
  }

  @Override
  public void periodic() {
    // Enables Dashboard Numbers to be updated each loop
    DashboardNumber.checkAll();
    Logger.recordOutput("OperatorMode", m_isAlgaeMode.getAsBoolean() ? Color.kSeaGreen.toHexString() : Color.kWhite.toHexString());
    Logger.recordOutput("ReefState", m_arm.getSelectedCoralState());
   
  }

  /**
   * Simple setter method.
   *
   * @param accept the pose or not
   */
  public void setSwerveDriveAcceptingVisionUpdates(boolean accept) {
    m_swerveDrive.setOdometryAcceptVisionData(accept);
  }

  /**
   * Run when Autonomous and Tele-op start.
   */
  public void autoAndTeleInit() {
    m_arm.changeState(ArmState.START);
  }

  /**
  * Sets motor to cleanup when disabled.
  */
  public void setMotorCleanUp() {
    m_arm.setMotorCleanUp();
  }

  /**
  * Sets motor to Start when enabled.
  */
  public void setMotorStartUp() {
    m_arm.setMotorStartUp();
  }

  /**
   * Attempts to update the pose estimator within the swerve drive object. Note that the SwerveDrive
   * may disregard pose updates as well.
   *
   * @param data VisionData structure containing the required parts
   */
  @Override
  public synchronized void updatePoseEstimator(VisionData data) {
    var pose = data.getPose2d();
    boolean check = true;
    if (pose == null
        || !Double.isFinite(pose.getX())
        || !Double.isFinite(pose.getY())
        || !Double.isFinite(pose.getRotation().getDegrees())) {
      check = false;
    }

    if (m_arm.getArmValues().basePivotAngle.in(Degrees) < 60.0) {
      check = false;
    }

    if (data.getConfidence() <= VisionConstants.limeLight3GSpecs.confidence_requirement) {
      check = false;
    }

    var pose3D = data.getPose3d();
    if (pose.getX() <= 0.0 || pose.getY() <= 0.0 || pose3D.getZ() <= -0.10 || pose3D.getZ() >= 0.30) {
      check = false;
    }
    if (check) {
      data.m_time -= VisionConstants.timeOffset;
      Logger.recordOutput("LimelightLatency", data.m_time);
      m_swerveDrive.addVisionMeasurement(data);
      Logger.recordOutput(data.getName() + "/accepted pose", pose);
    } else {
      Logger.recordOutput(data.getName() + "/rejected pose", pose);
    }
    Logger.recordOutput(data.getName() + "/PreviousPose", m_swerveDrive.getPastPose(data.m_time).get());
  }
}
