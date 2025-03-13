// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.robot.ChaosRobotContainer;
import com.chaos131.util.DashboardNumber;
import com.chaos131.vision.VisionData;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Constants.FieldDimensions;
import frc.robot.Constants.ArmConstants.ArmPoses;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.ChangeState;
import frc.robot.commands.DriverRelativeDrive;
import frc.robot.commands.IkScoring;
import frc.robot.commands.DriverRelativeSetAngleDrive;
import frc.robot.commands.ReefAlignment;
import frc.robot.commands.UpdateHeading;
import frc.robot.commands.WaitForCoral;
import frc.robot.commands.WaitForState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.MechManager2D;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Gripper;
import frc.robot.subsystems.arm.SelectedArmState;
import frc.robot.subsystems.arm.Arm.ArmState;
import frc.robot.utils.DriveDirection;
import frc.robot.utils.FieldPoint;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.HashMap;
import java.util.Map;

import org.ejml.data.FEigenpair;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
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
  public static Intake m_intake;
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
  private SelectedArmState m_selectedArmState = SelectedArmState.L4;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() throws Exception {
    super();
    m_gyro = new Pigeon2(CanIdentifiers.GyroCANID, CanIdentifiers.CTRECANBus);
    m_swerveDrive = SwerveDrive.createSwerveDrive(m_gyro);
    m_arm = new Arm(m_operator);
    m_intake = new Intake();
    m_mech2dManager = new MechManager2D(m_arm, m_intake);
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
    NamedCommands.registerCommand("IntakeFromHP", new ChangeState().setArm(ArmState.INTAKE_FROM_HP).andThen(new WaitForState().forArmState(ArmState.HOLD_CORAL)));
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
  }

  private void configureBindings() {
    m_swerveDrive.setDefaultCommand(new DriverRelativeDrive(m_driver, m_swerveDrive)); 

    // Everything after this is for competition
    m_driver.a().whileTrue(new DriverRelativeSetAngleDrive(m_driver, m_swerveDrive,  () -> {
      FieldPoint pose = FieldPoint.getNearestPoint(m_swerveDrive.getPose(), FieldPoint.getHpDrivePoses());
      return pose.getCurrentAlliancePose().getRotation();
    }, 1.0));
    // m_driver.a().whileTrue(PathUtil.driveToClosestPointCommand(FieldPoint.getHpDrivePoses(), m_swerveDrive)
    //     .alongWith(new ChangeState().setArm(ArmState.INTAKE_FROM_HP)
    //     .withArmInterrupt(ArmState.STOW)));
        
    m_driver.b().whileTrue(new DriverRelativeSetAngleDrive(m_driver, m_swerveDrive, () -> DriveDirection.Right.getAllianceAngle(), 1.0));
    m_driver.x().whileTrue(new DriverRelativeSetAngleDrive(m_driver, m_swerveDrive, () -> DriveDirection.Away.getAllianceAngle(), 1.0));
    // m_driver.y().whileTrue(new DriverRelativeSetAngleDrive(m_driver, m_swerveDrive,  () -> {
    //   FieldPoint pose = FieldPoint.getNearestPoint(m_swerveDrive.getPose(), FieldPoint.getReefDrivePoses());
    //   return pose.getCurrentAlliancePose().getRotation();
    // }, 1.0));
    m_driver.y().whileTrue(PathUtil.driveToClosestPointTeleopCommand(FieldPoint.getReefDrivePoses(), m_swerveDrive));
    // .andThen(new ChangeState().setArm(() -> m_selectedArmState.ScoreState).withArmInterrupt(ArmState.HOLD_CORAL))

    m_driver.povUp().onTrue(new UpdateHeading(m_swerveDrive, DriveDirection.Away)); // 0 degrees for blue
    m_driver.povDown().onTrue(new UpdateHeading(m_swerveDrive, DriveDirection.Towards)); // 180 degrees for blue
    m_driver.povLeft().onTrue(new UpdateHeading(m_swerveDrive, DriveDirection.Left)); // 90 degrees for blue
    m_driver.povRight().onTrue(new UpdateHeading(m_swerveDrive, DriveDirection.Right)); // -90 degrees for blue

    m_driver.start().whileTrue(new ChangeState().setArm(ArmState.POST_CLIMB));
    m_driver.back().whileTrue(new ChangeState().setArm(ArmState.PREP_CLIMB));

    m_driver.rightBumper().or(m_driver.rightTrigger()).whileTrue(new StartEndCommand(() -> m_swerveDrive.setRampRatePeriod(SwerveConstants.DriverSlowRampRatePeriod), () -> m_swerveDrive.setRampRatePeriod(SwerveConstants.DriverRampRatePeriod)));
    m_driver.rightBumper().whileTrue(new ChangeState().setArm(() -> m_selectedArmState.PrepState).withArmInterrupt(ArmState.HOLD_CORAL));
    m_driver.rightTrigger().whileTrue(new ChangeState().setArm(() -> m_selectedArmState.ScoreState).withArmInterrupt(ArmState.HOLD_CORAL));
    m_driver.leftTrigger().whileTrue(new ChangeState().setArm(ArmState.INTAKE_FROM_HP).withArmInterrupt(ArmState.STOW));
    m_driver.leftBumper().whileTrue(new ChangeState().setArm(() -> {
      var closestTag = FieldPoint.getNearestPoint(m_swerveDrive.getPose(), FieldPoint.getReefAprilTagPoses());
      return m_aprilTagToAlgaeHeightMap.get(closestTag.getName());
    }).withArmInterrupt(ArmState.STOW));

    m_operator.a().onTrue(new InstantCommand(() -> m_selectedArmState = SelectedArmState.L1));
    m_operator.x().onTrue(new InstantCommand(() -> m_selectedArmState = SelectedArmState.L2));
    m_operator.b().onTrue(new InstantCommand(() -> m_selectedArmState = SelectedArmState.L3));
    m_operator.y().onTrue(new InstantCommand(() -> m_selectedArmState = SelectedArmState.L4));

    m_operator.povUp().whileTrue(new ChangeState().setArm(ArmState.PREP_CLIMB));
    m_operator.povUp().whileTrue(new ChangeState().setArm(ArmState.POST_CLIMB));
    
    m_operator.start().onTrue(new ChangeState().setArm(ArmState.STOW));
    m_operator.back().whileTrue(new ChangeState().setArm(ArmState.MANUAL));
    //.alongWith(new InstantCommand(() -> m_arm.m_gripperPivot.disableFuseCANcoder()))

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

    // m_operator.start().whileTrue(new ChangeState().setArm(ArmState.STOW).setIntake(IntakeState.STOW));
    // m_operator.leftBumper().whileTrue(new ChangeState().setArm(ArmState.INTAKE_FROM_FLOOR).setIntake(IntakeState.DEPLOY));
    // m_operator.rightBumper().whileTrue(new ChangeState().setArm(ArmState.INTAKE_FROM_HP).setIntake(IntakeState.STOW));
    // m_operator.back().onTrue(new InstantCommand(() -> Gripper.hasCoralGrippedSim = !Gripper.hasCoralGrippedSim)); // TODO: delete if back button needed for competition
    // m_operator.povLeft().whileTrue(new ChangeState().setArm(ArmState.MANUAL).setIntake(IntakeState.STOW));

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
    var lower_branch_angle = Degrees.of(35).in(Radians);
    // It would normally be [0, pi/2, pi] rotations, but that's outside of our range... so lets go with something like... 60 deg? (positive down)
    // We use 180 degree yaw to turn the robot around. We take the April Tag orientation, but we need an orientation that makes sense for the robot
    Transform3d branchLeftR4Offset = new Transform3d(FieldDimensions.ReefBranchLeft.getX(),
                                                      FieldDimensions.ReefBranchLeft.getY(),
                                                      FieldDimensions.Reef4Meters,
                                                      new Rotation3d(0, Degrees.of(60).in(Radians), Math.PI));
    Pose3d branchLeftR4 = new Pose3d(FieldPoint.aprilTagMap.get(17).pose2d).transformBy(branchLeftR4Offset);
    m_driver.rightTrigger().whileTrue(new IkScoring(m_driver, m_swerveDrive, m_idLift, branchLeftR4));

    Transform3d branchLeftR3Offset = new Transform3d(FieldDimensions.ReefBranchLeft.getX(),
                                                      FieldDimensions.ReefBranchLeft.getY(),
                                                      FieldDimensions.Reef3Meters,
                                                      new Rotation3d(0, lower_branch_angle, Math.PI));
    Pose3d branchLeftR3 = new Pose3d(FieldPoint.aprilTagMap.get(17).pose2d).transformBy(branchLeftR3Offset);
    m_driver.rightBumper().whileTrue(new IkScoring(m_driver, m_swerveDrive, m_idLift, branchLeftR3));
    
    Transform3d branchLeftR2Offset = new Transform3d(FieldDimensions.ReefBranchLeft.getX(),
                                                      FieldDimensions.ReefBranchLeft.getY(),
                                                      FieldDimensions.Reef2Meters,
                                                      new Rotation3d(0, lower_branch_angle, Math.PI));
    Pose3d branchLeftR2 = new Pose3d(FieldPoint.aprilTagMap.get(17).pose2d).transformBy(branchLeftR2Offset);
    m_driver.leftBumper().whileTrue(new IkScoring(m_driver, m_swerveDrive, m_idLift, branchLeftR2));
    
    Transform3d branchLeftR1Offset = new Transform3d(FieldDimensions.ReefBranchLeft.getX(),
                                                      FieldDimensions.ReefBranchLeft.getY(),
                                                      FieldDimensions.Reef1Meters,
                                                      new Rotation3d(0, 0, Math.PI));
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

  public void setSwerveDriveAcceptingVisionUpdates(boolean accept) {
    m_swerveDrive.setOdometryAcceptVisionData(accept);
  }

  public void autoAndTeleInit(){
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

  @Override
   /**
   * Attempts to update the pose estimator within the swerve drive object. Note that the SwerveDrive
   * may disregard pose updates as well.
   *
   * @param data VisionData structure containing the required parts
   */
  public synchronized void updatePoseEstimator(VisionData data) {
    var pose = data.getPose2d();
    var pose3D = data.getPose3d();
    boolean check = true;
    if (pose == null
        || !Double.isFinite(pose.getX())
        || !Double.isFinite(pose.getY())
        || !Double.isFinite(pose.getRotation().getDegrees())) {
      check = false;
    }

    if (m_arm.getArmValues().basePivotAngle.getDegrees() < 60.0) {
      check = false;
    }

    if (data.getConfidence() <= VisionConstants.limeLight3GSpecs.confidence_requirement){
      check = false;
    }

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
   
  }
}
