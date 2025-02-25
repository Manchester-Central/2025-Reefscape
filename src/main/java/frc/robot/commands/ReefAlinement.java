package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldDimensions;
import frc.robot.Constants.RobotDimensions;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.FieldPoint;

public class ReefAlinement extends Command{

  private FieldPoint m_aprilTag;
  private boolean m_isInvert;
  private SwerveDrive m_swerveDrive;

  private Pose2d branchPostion;

  public ReefAlinement(FieldPoint aprilTag, boolean isInvert, SwerveDrive swerveDrive){
    m_aprilTag = aprilTag;
    m_isInvert = isInvert;
    m_swerveDrive = swerveDrive;
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    m_swerveDrive.driverModeInit();
    int invertValue = m_isInvert ? -1 : 1;

    branchPostion = m_aprilTag.getCurrentAlliancePose().plus(new Transform2d(
      (RobotDimensions.FrontBackLengthMeters / 2) + RobotDimensions.RobotToReefMargin, 
      FieldDimensions.ReefBranchRight.getY() * invertValue, 
      Rotation2d.fromDegrees(180)));

      m_swerveDrive.setTarget(branchPostion);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveDrive.moveToTarget(.5); // TODO: change when we know it works/ in sim
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.setXMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_swerveDrive.atTarget();
  }
}
