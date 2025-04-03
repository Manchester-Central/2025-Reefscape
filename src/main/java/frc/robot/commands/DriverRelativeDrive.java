// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.swerve.BaseSwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SwerveConstants;

/**
 * A class to drive the robot in driver relative mode.
 */
public class DriverRelativeDrive extends Command {
  /** Creates a new DriverRelativeDrive. */
  Gamepad m_driver;
  Trigger m_slowModeTrigger;

  BaseSwerveDrive m_swerveDrive;

  /**
   * Creates a new DriverRelativeDrive.
   */
  public DriverRelativeDrive(Gamepad driver, BaseSwerveDrive swerve, Trigger slowModeTrigger) {
    m_driver = driver;
    m_swerveDrive = swerve;
    m_slowModeTrigger = slowModeTrigger;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveDrive.driverModeInit();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedMod = m_slowModeTrigger.getAsBoolean() ? 0.3 : 1.0;
    // double speedMod = m_driver.leftStick().getAsBoolean() 
    // || m_driver.rightTrigger().getAsBoolean()
    // || m_driver.rightBumper().getAsBoolean() 
    // || m_driver.leftTrigger().getAsBoolean()
    m_swerveDrive.moveFieldRelative(
        SwerveConstants.MaxFreeSpeed.times(m_driver.getSlewLeftY() * speedMod), 
        SwerveConstants.MaxFreeSpeed.times(-m_driver.getSlewLeftX() * speedMod), 
        SwerveConstants.MaxRotationSpeed.times(-m_driver.getSlewRightX() * speedMod));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
