// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.robot.ChaosRobotContainer;
import com.chaos131.util.DashboardNumber;
import com.chaos131.vision.LimelightCamera.LimelightVersion;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.ChangeState;
import frc.robot.commands.DriverRelativeDrive;
import frc.robot.commands.SimpleDriveToPosition;
import frc.robot.commands.UpdateHeading;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Mech2DManager;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.lift.Gripper;
import frc.robot.subsystems.lift.IdLift;
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
  public static Camera m_rightCamera;
  public static Camera m_leftCamera;
  public static Mech2DManager m_mech2dManager;
  public static SwerveDriveSimulation m_driveSim;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   *
   * @throws Exception
   */
  public RobotContainer() throws Exception {
    super();
    m_gyro = new Pigeon2(CanIdentifiers.GyroCANID, CanIdentifiers.CTRECANBus);
    m_swerveDrive = SwerveDrive.SeparateConstructor(m_gyro);
    m_idLift = new IdLift();
    m_intake = new Intake();
    m_mech2dManager = new Mech2DManager(m_idLift, m_intake);
    m_rightCamera =
        new Camera(
            "limelight-right",
            LimelightVersion.LL3G,
            VisionConstants.limeLight3GSpecs,
            () -> m_swerveDrive.getPose(),
            (data) -> updatePoseEstimator(data),
            () -> m_swerveDrive.getRobotSpeedMps(),
            () -> m_swerveDrive.getRobotRotationSpeedRadsPerSec());
    m_leftCamera =
        new Camera(
            "limelight-left",
            LimelightVersion.LL3G,
            VisionConstants.limeLight3GSpecs,
            () -> m_swerveDrive.getPose(),
            (data) -> updatePoseEstimator(data),
            () -> m_swerveDrive.getRobotSpeedMps(),
            () -> m_swerveDrive.getRobotRotationSpeedRadsPerSec());
    buildPathplannerAutoChooser();
    // Configure the trigger bindings
    configureBindings();
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
    m_swerveDrive.setDefaultCommand(new DriverRelativeDrive(m_driver, m_swerveDrive));

    m_driver.a().whileTrue(new SimpleDriveToPosition(m_swerveDrive, FieldPoint.leftSource));
    m_driver.b().whileTrue(m_swerveDrive.followPathCommand("Test Path"));
    m_driver.y().whileTrue(PathUtil.toCreateFindAPathCommand(FieldPoint.testPoint, m_swerveDrive));
    m_driver
        .x()
        .whileTrue(
            PathUtil.toCreateFindAPathToClosestPointCommand(
                FieldPoint.getReefDrivePoses(), m_swerveDrive));
    m_driver
        .povUp()
        .onTrue(new UpdateHeading(m_swerveDrive, DriveDirection.Away)); // 0 degrees for blue
    m_driver
        .povDown()
        .onTrue(new UpdateHeading(m_swerveDrive, DriveDirection.Towards)); // 180 degrees for blue
    m_driver
        .povLeft()
        .onTrue(new UpdateHeading(m_swerveDrive, DriveDirection.Left)); // 90 degrees for blue
    m_driver
        .povRight()
        .onTrue(new UpdateHeading(m_swerveDrive, DriveDirection.Right)); // -90 degrees for blue

    m_simKeyboard
        .y() // v on keyboard 0
        .onTrue(
            new InstantCommand(
                () -> {
                  System.out.println("Adding Game Piece");
                  SimulatedArena.getInstance()
                      .addGamePiece(
                          new ReefscapeCoralOnField(new Pose2d(2, 1, Rotation2d.fromDegrees(90))));
                }));
    m_simKeyboard
        .a() // z on keyboard 0
        .onTrue(
            new InstantCommand(
                () -> Gripper.hasCoralFrontGrippedSim = !Gripper.hasCoralFrontGrippedSim));

    m_simKeyboard
        .b() // x on keyboard 0
        .onTrue(
            new InstantCommand(
                () -> Gripper.hasCoralBackGrippedSim = !Gripper.hasCoralBackGrippedSim));
    m_operator
        .start()
        .whileTrue(new ChangeState().setLift(LiftState.STOW).setIntake(IntakeState.STOW));
    m_operator
        .leftBumper()
        .whileTrue(
            new ChangeState().setLift(LiftState.INTAKE_FROM_FLOOR).setIntake(IntakeState.DEPLOY));
    // m_operator.rightBumper().whileTrue(new ChangeState().setLift(LiftState.INTAKE_FROM_HP));
    m_operator
        .a()
        .whileTrue(new ChangeState().setLift(LiftState.SCORE_L1).setIntake(IntakeState.STOW));
    m_operator
        .x()
        .whileTrue(new ChangeState().setLift(LiftState.SCORE_L2).setIntake(IntakeState.STOW));
    m_operator
        .b()
        .whileTrue(new ChangeState().setLift(LiftState.SCORE_L3).setIntake(IntakeState.STOW));
    m_operator
        .y()
        .whileTrue(new ChangeState().setLift(LiftState.SCORE_L4).setIntake(IntakeState.STOW));
  }

  @Override
  public void configureDriverController() {
    m_driver = new Gamepad(0);
  }

  @Override
  public void configureOperatorController() {
    m_operator = new Gamepad(1);
  }

  @Override
  public void configureTesterController() {
    // TODO Auto-generated method stub

  }

  @Override
  public void configureSimKeyboard() {
    m_simKeyboard = new Gamepad(2);
    // TODO Auto-generated method stub

  }

  @Override
  public void periodic() {
    // Enables Dashboard Numbers to be updated each loop
    DashboardNumber.checkAll();
    // TODO Auto-generated method stub

  }
}
