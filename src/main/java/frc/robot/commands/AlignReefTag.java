// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldDimensions;
import frc.robot.Constants.RobotDimensions;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignReefTag extends Command {
  private SwerveDrive m_swerveDrive;
  private Camera m_cameraLeft;
  private Camera m_cameraRight;
  /** Creates a new AlignReefTag. */
  public AlignReefTag(SwerveDrive swerveDrive, Camera cameraLeft, Camera cameraRight) {
    m_swerveDrive = swerveDrive;
    m_cameraLeft = cameraLeft;
    m_cameraRight = cameraRight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose3d targetPoseLeft = m_cameraLeft.getTargetPose3d_RobotSpace();
    Pose3d targetPoseRight = m_cameraRight.getTargetPose3d_RobotSpace();
    boolean hasPoseLeft = targetPoseLeft.getX() == 0 && targetPoseLeft.getY() == 0;
    boolean hasPoseRight = targetPoseRight.getX() == 0 && targetPoseRight.getY() == 0;
    if (!hasPoseLeft || !hasPoseRight) {
      return;
    }
    
    double targetX = RobotDimensions.FrontBackLengthMeters / 2 + RobotDimensions.RobotToReefMargin;
    double targetY = 0.0;
    Angle targetAngle = Degrees.of(0);
    Pose3d poseToUse = null;
    if (!hasPoseRight) {
      // handle left
      targetY = FieldDimensions.ReefBranchLeft.getY();
      poseToUse = targetPoseLeft;
    } else if (!hasPoseLeft) {
      // handle right
      targetY = FieldDimensions.ReefBranchRight.getY();
      poseToUse = targetPoseRight;
    } else {
      // figure out if left or right is closes
      double distanceLeft = targetPoseLeft.toPose2d().getTranslation().getNorm();
      double distanceRight = targetPoseRight.toPose2d().getTranslation().getNorm();
      targetY = distanceLeft < distanceRight ? FieldDimensions.ReefBranchLeft.getY() : FieldDimensions.ReefBranchRight.getY();
      poseToUse = distanceLeft < distanceRight ? targetPoseRight : targetPoseLeft;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
