// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.swerve.BaseSwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.FieldPoint;

/**
 * A simple command to go to a position on the field using basic PID translation control.
 */
public class SimpleDriveToPosition extends Command {
  BaseSwerveDrive m_swerveDrive;
  FieldPoint m_fieldPoint;

  /** Creates a new SimpleDriveToPosition. */
  public SimpleDriveToPosition(BaseSwerveDrive swerveDrive, FieldPoint fieldPoint) {
    m_swerveDrive = swerveDrive;
    m_fieldPoint = fieldPoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveDrive.driveToPositionInit();
    m_swerveDrive.resetPids();
    m_swerveDrive.setTarget(m_fieldPoint.getCurrentAlliancePose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveDrive.moveToTarget(0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.resetPids();
    m_swerveDrive.driverModeInit();
    m_swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_swerveDrive.atTarget(0.01);
  }
}
