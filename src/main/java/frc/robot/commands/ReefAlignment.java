package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldDimensions;
import frc.robot.Constants.RobotDimensions;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.FieldPoint;

/**
 * Moves to a position on the reef nearest an april tag.
 */
public class ReefAlignment extends Command {

  private FieldPoint m_aprilTag;
  private boolean m_isInvert;
  private SwerveDrive m_swerveDrive;

  private Pose2d m_branchPostion;

  /**
   * Moves to a position on the reef nearest an april tag.
   *
   * @param aprilTag to align with
   * @param isInvert determines if the pose moves to the left (with respect to april tag orientation), false moves right
   * @param swerveDrive the swerve system to drive
   */
  public ReefAlignment(FieldPoint aprilTag, boolean isInvert, SwerveDrive swerveDrive) {
    m_aprilTag = aprilTag;
    m_isInvert = isInvert;
    m_swerveDrive = swerveDrive;
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    m_swerveDrive.driverModeInit();
    int invertValue = m_isInvert ? -1 : 1;

    m_branchPostion = m_aprilTag.getCurrentAlliancePose().plus(
        new Transform2d((RobotDimensions.FrontBackLength.in(Meters) / 2) + RobotDimensions.RobotToReefCoralMargin.in(Meters), 
                        FieldDimensions.ReefBranchRight.getY() * invertValue, 
                        Rotation2d.fromDegrees(180)));

    m_swerveDrive.setTarget(m_branchPostion);
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
