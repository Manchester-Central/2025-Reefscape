// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.FieldPoint;

/**
 * A simple command to go to a position on the field using basic PID translation control.
 */
public class SimpleDriveToPositionV2 extends Command {
  SwerveDrive m_swerveDrive;
  Supplier<Pose2d> m_poseSup;

  /** Creates a new SimpleDriveToPosition. */
  public SimpleDriveToPositionV2(SwerveDrive swerveDrive, FieldPoint fieldPoint) {
    m_swerveDrive = swerveDrive;
    m_poseSup = () -> fieldPoint.getBluePose();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  /** Creates a new SimpleDriveToPosition. */
  public SimpleDriveToPositionV2(SwerveDrive swerveDrive, Supplier<Pose2d> poseSup) {
    m_swerveDrive = swerveDrive;
    m_poseSup = poseSup;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveDrive.driveToPositionInit();
    m_swerveDrive.resetPids();
    m_swerveDrive.setTarget(m_poseSup.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveDrive.setTarget(m_poseSup.get());
    m_swerveDrive.moveToTargetV2(0.3);
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
    return m_swerveDrive.atTarget(0.01) && !DriverStation.isTeleop();
    //return ((SwerveDrive) m_swerveDrive).atTargetDynamic() && !DriverStation.isTeleop();
  }
}
