// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.swerve.BaseSwerveDrive;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Robot;
import frc.robot.subsystems.lift.IdLift;
import frc.robot.subsystems.lift.IdLift.LiftState;

/**
 * A class to drive the robot in driver relative mode.
 */
public class IkScoring extends Command {
  /** Creates a new DriverRelativeDrive. */
  Gamepad m_driver;
  BaseSwerveDrive m_swerveDrive;
  IdLift m_lift;
  Pose3d m_endEffectorPose;

  /**
   * Creates a new DriverRelativeDrive.
   */
  public IkScoring(Gamepad driver, BaseSwerveDrive swerve, IdLift lift, Pose3d endEffectorPose) {
    m_driver = driver;
    m_swerveDrive = swerve;
    m_lift = lift;
    m_endEffectorPose = endEffectorPose;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lift.setIkSolverTarget(m_endEffectorPose);
    m_lift.changeState(LiftState.IKSOLVER);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var angle = m_endEffectorPose.toPose2d().getTranslation().minus(m_swerveDrive.getPose().getTranslation()).getAngle();
    m_swerveDrive.moveFieldRelativeAngle(
        SwerveConstants.MaxFreeSpeed.times(m_driver.getSlewLeftY()), 
        SwerveConstants.MaxFreeSpeed.times(-m_driver.getSlewLeftX()),
        angle, 1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Robot.isSimulation()) {
      return false;
    }
    return !m_lift.m_gripper.hasCoral();
  }
}
