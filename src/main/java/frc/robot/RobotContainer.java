// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.robot.ChaosRobotContainer;
import com.chaos131.util.DashboardNumber;
import com.chaos131.vision.LimelightCamera.LimelightVersion;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Constants.FieldDimensions;
import frc.robot.Constants.MidLiftConstants.LiftPoses;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.ChangeState;
import frc.robot.commands.DriverRelativeDrive;
import frc.robot.commands.IkScoring;
import frc.robot.commands.DriverRelativeSetAngleDrive;
import frc.robot.commands.SimpleDriveToPosition;
import frc.robot.commands.UpdateHeading;
import frc.robot.commands.WaitForState;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.MechManager2D;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.lift.Gripper;
import frc.robot.subsystems.lift.IdLift;
import frc.robot.subsystems.lift.LiftPose;
import frc.robot.subsystems.lift.SelectedLiftState;
import frc.robot.subsystems.lift.IdLift.LiftState;
import frc.robot.utils.DriveDirection;
import frc.robot.utils.FieldPoint;
import frc.robot.utils.PathUtil;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer extends ChaosRobotContainer<SwerveDrive> {

  Pigeon2 m_gyro;

  public static IdLift m_idLift;
  public static Intake m_intake;
  //public static Camera m_rightCamera;
  //public static Camera m_leftCamera;
  public static MechManager2D m_mech2dManager;
  public static SwerveDriveSimulation m_driveSim;
  private SelectedLiftState m_selectedLiftState = SelectedLiftState.L4;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() throws Exception {
    super();
    m_gyro = new Pigeon2(CanIdentifiers.GyroCANID, CanIdentifiers.CTRECANBus);
    m_swerveDrive = SwerveDrive.createSwerveDrive(m_gyro);
    m_idLift = new IdLift(m_operator, () -> m_swerveDrive.getPose());
    m_intake = new Intake();
    m_mech2dManager = new MechManager2D(m_idLift);
    // m_rightCamera =
    //     new Camera(
    //         "limelight-right",
    //         LimelightVersion.LL3G,
    //         VisionConstants.limeLight3GSpecs,
    //         () -> m_swerveDrive.getPose(),
    //         (data) -> updatePoseEstimator(data),
    //         () -> m_swerveDrive.getRobotSpeed().in(MetersPerSecond),
    //         () -> m_swerveDrive.getRobotRotationSpeed().in(RadiansPerSecond));
    // m_leftCamera =
    //     new Camera(
    //         "limelight-left",
    //         LimelightVersion.LL3G,
    //         VisionConstants.limeLight3GSpecs,
    //         () -> m_swerveDrive.getPose(),
    //         (data) -> updatePoseEstimator(data),
    //         () -> m_swerveDrive.getRobotSpeed().in(MetersPerSecond),
    //         () -> m_swerveDrive.getRobotRotationSpeed().in(RadiansPerSecond));
    
    NamedCommands.registerCommand("ScoreL4",  new ChangeState().setLift(LiftState.SCORE_L4).setIntake(IntakeState.STOW).andThen(new WaitForState().forLiftState(LiftState.STOW)));
    NamedCommands.registerCommand("IntakeFromHP", new ChangeState().setLift(LiftState.INTAKE_FROM_HP).setIntake(IntakeState.STOW).andThen(new WaitForState().forLiftState(LiftState.STOW)));
    NamedCommands.registerCommand("XMode", new RunCommand(() -> m_swerveDrive.setXMode()).withTimeout(0.1));
    buildPathplannerAutoChooser();


    
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    m_swerveDrive.setDefaultCommand(new DriverRelativeDrive(m_driver, m_swerveDrive)); 

    // Everything after this is for competition
    m_driver.a().whileTrue(new DriverRelativeSetAngleDrive(m_driver, m_swerveDrive,  () -> {
      FieldPoint pose = FieldPoint.getNearestPoint(m_swerveDrive.getPose(), FieldPoint.getHpDrivePoses());
      return pose.getCurrentAlliancePose().getRotation();
      // return pose.getCurrentAlliancePose().getTranslation().minus(m_swerveDrive.getPose().getTranslation()).getAngle();
    }, 1.0));
    m_driver.povUp().onTrue(new UpdateHeading(m_swerveDrive, DriveDirection.Away)); // 0 degrees for blue
    m_driver.povDown().onTrue(new UpdateHeading(m_swerveDrive, DriveDirection.Towards)); // 180 degrees for blue
    m_driver.povLeft().onTrue(new UpdateHeading(m_swerveDrive, DriveDirection.Left)); // 90 degrees for blue
    m_driver.povRight().onTrue(new UpdateHeading(m_swerveDrive, DriveDirection.Right)); // -90 degrees for blue

    m_driver.rightBumper().whileTrue(new ChangeState().setLift(() -> m_selectedLiftState.PrepState));
    m_driver.rightTrigger().whileTrue(new ChangeState().setLift(() -> m_selectedLiftState.ScoreState));

    m_operator.a().onTrue(new InstantCommand(() -> m_selectedLiftState = SelectedLiftState.L1));
    m_operator.x().onTrue(new InstantCommand(() -> m_selectedLiftState = SelectedLiftState.L2));
    m_operator.b().onTrue(new InstantCommand(() -> m_selectedLiftState = SelectedLiftState.L3));
    m_operator.y().onTrue(new InstantCommand(() -> m_selectedLiftState = SelectedLiftState.L4));

    m_operator.povUp().whileTrue(new RunCommand(() -> {
      // m_idLift.m_extender.setTargetLength(LiftPoses.ClimbPrep.getExtensionMeters());
      m_idLift.m_basePivot.setTargetAngle(LiftPoses.ClimbPrep.getBasePivotAngle());
    }, m_idLift));
    m_operator.povDown().whileTrue(new RunCommand(() -> {
      // m_idLift.m_extender.setTargetLength(LiftPoses.Climb.getExtensionMeters());
      m_idLift.m_basePivot.setTargetAngle(LiftPoses.Climb.getBasePivotAngle());
    }, m_idLift));

    // Everything after this is for demos and testing
    // m_driver.a().whileTrue(new SimpleDriveToPosition(m_swerveDrive, FieldPoint.leftSource));
    // m_driver.b().whileTrue(m_swerveDrive.followPathCommand("Test Path"));
    // m_driver.y().whileTrue(PathUtil.driveToClosestPointCommand(FieldPoint.getHpDrivePoses(), m_swerveDrive));
    // m_driver.x().whileTrue(PathUtil.driveToClosestPointCommand(FieldPoint.getReefDrivePoses(), m_swerveDrive));



    // v on keyboard 0
    m_simKeyboard.y().onTrue(
      new InstantCommand(() -> {
        System.out.println("Adding Game Piece");
        SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(new Pose2d(2, 1, Rotation2d.fromDegrees(90))));
      })
    );
    // z on keyboard 0
    m_simKeyboard.a().onTrue(new InstantCommand(() -> Gripper.hasCoralGrippedSim = !Gripper.hasCoralGrippedSim));

    // m_operator.start().whileTrue(new ChangeState().setLift(LiftState.STOW).setIntake(IntakeState.STOW));
    // m_operator.leftBumper().whileTrue(new ChangeState().setLift(LiftState.INTAKE_FROM_FLOOR).setIntake(IntakeState.DEPLOY));
    // m_operator.rightBumper().whileTrue(new ChangeState().setLift(LiftState.INTAKE_FROM_HP).setIntake(IntakeState.STOW));
    // m_operator.back().onTrue(new InstantCommand(() -> Gripper.hasCoralGrippedSim = !Gripper.hasCoralGrippedSim)); // TODO: delete if back button needed for competition
    // m_operator.povLeft().whileTrue(new ChangeState().setLift(LiftState.MANUAL).setIntake(IntakeState.STOW));

    // m_operator.leftTrigger().whileTrue(new RunCommand(() -> m_idLift.m_gripperPivot.setTargetAngle(Rotation2d.fromDegrees(-20)),
    //     m_idLift));
    // m_operator.rightTrigger().whileTrue(new RunCommand(() -> m_idLift.m_gripperPivot.setTargetAngle(Rotation2d.fromDegrees(-90)),
    //     m_idLift));
    // m_operator.a().whileTrue(new RunCommand(() -> m_idLift.m_extender.setTargetLength(0.5), m_idLift));
    // m_operator.b().whileTrue(new RunCommand(() -> m_idLift.m_extender.setTargetLength(1.2), m_idLift));
    // m_operator.povUp().whileTrue(new RunCommand(() -> {
    //   m_idLift.m_extender.setTargetLength(LiftPoses.ClimbPrep.getExtensionMeters());
    //   m_idLift.m_basePivot.setTargetAngle(LiftPoses.ClimbPrep.getBasePivotAngle());
    // }, m_idLift));
    // m_operator.povDown().whileTrue(new RunCommand(() -> {
    //   m_idLift.m_extender.setTargetLength(LiftPoses.Climb.getExtensionMeters());
    //   m_idLift.m_basePivot.setTargetAngle(LiftPoses.Climb.getBasePivotAngle());
    // }, m_idLift));

    // We flatten the april tag onto the ground, so we can use the Reef3Meters height while keeping the floor angle
    Transform3d branchLeftR4Offset = new Transform3d(FieldDimensions.ReefBranchLeft.getX(),
                                                      FieldDimensions.ReefBranchLeft.getY(),
                                                      FieldDimensions.Reef4Meters,
                                                      new Rotation3d(0, Math.PI / 2, 0));
    Pose3d branchLeftR4 = new Pose3d(FieldPoint.aprilTagMap.get(17).pose2d).transformBy(branchLeftR4Offset);
    m_driver.rightTrigger().whileTrue(new IkScoring(m_driver, m_swerveDrive, m_idLift, branchLeftR4));

    Transform3d branchLeftR3Offset = new Transform3d(FieldDimensions.ReefBranchLeft.getX(),
                                                      FieldDimensions.ReefBranchLeft.getY(),
                                                      FieldDimensions.Reef3Meters,
                                                      new Rotation3d(0, Math.PI / 2, 0));
    Pose3d branchLeftR3 = new Pose3d(FieldPoint.aprilTagMap.get(17).pose2d).transformBy(branchLeftR3Offset);
    m_driver.rightBumper().whileTrue(new IkScoring(m_driver, m_swerveDrive, m_idLift, branchLeftR3));
    
    Transform3d branchLeftR2Offset = new Transform3d(FieldDimensions.ReefBranchLeft.getX(),
                                                      FieldDimensions.ReefBranchLeft.getY(),
                                                      FieldDimensions.Reef2Meters,
                                                      new Rotation3d(0, Math.PI / 2, 0));
    Pose3d branchLeftR2 = new Pose3d(FieldPoint.aprilTagMap.get(17).pose2d).transformBy(branchLeftR2Offset);
    m_driver.leftBumper().whileTrue(new IkScoring(m_driver, m_swerveDrive, m_idLift, branchLeftR2));
                                                     
    Transform3d branchLeftR1Offset = new Transform3d(FieldDimensions.ReefBranchLeft.getX(),
                                                      FieldDimensions.ReefBranchLeft.getY(),
                                                      FieldDimensions.Reef1Meters,
                                                      new Rotation3d(0, Math.PI / 2, 0));
    Pose3d branchLeftR1 = new Pose3d(FieldPoint.aprilTagMap.get(17).pose2d).transformBy(branchLeftR1Offset);
    m_driver.leftTrigger().whileTrue(new IkScoring(m_driver, m_swerveDrive, m_idLift, branchLeftR1));
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
  }

  /**
  * Sets motor to cleanup when disabled.
  */
  public void setMotorCleanUp() {
    m_idLift.setMotorCleanUp();
  }

  /**
  * Sets motor to Start when enabled.
  */
  public void setMotorStartUp() {
    m_idLift.setMotorStartUp();
  }
}
