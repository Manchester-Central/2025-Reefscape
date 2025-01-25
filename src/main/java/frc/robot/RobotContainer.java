// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.robot.ChaosRobotContainer;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ChangeState;
import frc.robot.commands.DriverRelativeDrive;
import frc.robot.commands.SimpleDriveToPosition;
import frc.robot.subsystems.FrontCamera;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Mech2DManager;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.lift.IdLift;
import frc.robot.subsystems.lift.IdLift.LiftState;
import frc.robot.utils.FieldPoint;

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
  public static FrontCamera m_frontcamera;
  public static Mech2DManager m_mech2dManager;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   *
   * @throws Exception
   */
  public RobotContainer() throws Exception {
    super();
    m_gyro = new Pigeon2(Constants.GyroConstants.GyroCANID);
    m_swerveDrive = SwerveDrive.SeparateConstructor(m_gyro);
    m_idLift = new IdLift();
    m_intake = new Intake();
    m_mech2dManager = new Mech2DManager(m_idLift, m_intake);
    m_frontcamera = new FrontCamera();
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

    m_operator.start().whileTrue(new ChangeState().setLift(LiftState.STOW));
    m_operator.leftBumper().whileTrue(new ChangeState().setLift(LiftState.INTAKE_FROM_FLOOR));
    m_operator.rightBumper().whileTrue(new ChangeState().setLift(LiftState.INTAKE_FROM_HP));
    m_operator.a().whileTrue(new ChangeState().setLift(LiftState.SCORE_L1));
    m_operator.x().whileTrue(new ChangeState().setLift(LiftState.SCORE_L2));
    m_operator.b().whileTrue(new ChangeState().setLift(LiftState.SCORE_L3));
    m_operator.y().whileTrue(new ChangeState().setLift(LiftState.SCORE_L4));
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
    // TODO Auto-generated method stub

  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub

  }
}
