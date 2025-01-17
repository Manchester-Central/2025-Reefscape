// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.robot.ChaosRobotContainer;
import com.chaos131.swerve.BaseSwerveDrive;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriverRelativeDrive;
import frc.robot.commands.SimpleDriveToPosition;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.utils.FieldPoint;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.*;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer extends ChaosRobotContainer {

  Pigeon2 m_gyro;

  private Manipulator m_manipulator;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
      * @throws Exception 
      */
     public RobotContainer() throws Exception {
    super();
    m_gyro = new Pigeon2(Constants.GyroConstants.GyroCANID);
    m_swerveDrive = SwerveDrive.SeparateConstructor(m_gyro);
    m_manipulator = new Manipulator();
    BuildAutoer();
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_swerveDrive.setDefaultCommand(new DriverRelativeDrive(m_driver, m_swerveDrive));

    m_driver.a().whileTrue(new SimpleDriveToPosition(m_swerveDrive, FieldPoint.leftSource));

    m_operator.a().whileTrue(new RunCommand(()-> m_manipulator.m_lift.setSpeed(0.5), m_manipulator.m_lift));
    m_operator.b().whileTrue(new RunCommand(()-> m_manipulator.m_lift.setSpeed(-0.5), m_manipulator.m_lift));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  @Override
  public void configureDriverController() {
    // TODO Auto-generated method stub
    m_driver = new Gamepad(0);
  }

  @Override
  public void configureOperatorController() {
    // TODO Auto-generated method stub
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
