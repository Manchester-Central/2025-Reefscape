// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldDimensions;
import frc.robot.Constants.RobotDimensions;
import frc.robot.Robot;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.FieldPoint;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 *  You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands.
 */
public class AlignReefTag extends Command {
  private SwerveDrive m_swerveDrive;
  private Camera m_cameraLeft;
  private Camera m_cameraRight;
  private Angle m_targetAngle;

  private static double kTranslationP = 2.0; // TODO: dashboard numbers
  private static double kTranslationI = 0.01;
  private static double kTranslationD = 0.0;
  private static LinearVelocity kMaxVelocity = MetersPerSecond.of(2.0);
  private static LinearAcceleration kMaxAccel = MetersPerSecondPerSecond.of(2.0);
  private static Constraints kConstraints = new Constraints(kMaxVelocity.in(MetersPerSecond), kMaxAccel.in(MetersPerSecondPerSecond));
  private static Distance kTranslationTolerance = Meters.of(0.01);

  private ProfiledPIDController m_pidX = new ProfiledPIDController(kTranslationP, kTranslationI, kTranslationD, kConstraints);
  private ProfiledPIDController m_pidY = new ProfiledPIDController(kTranslationP, kTranslationI, kTranslationD, kConstraints);

  /** Creates a new AlignReefTag. */
  public AlignReefTag(SwerveDrive swerveDrive, Camera cameraLeft, Camera cameraRight) {
    m_swerveDrive = swerveDrive;
    m_cameraLeft = cameraLeft;
    m_cameraRight = cameraRight;
    m_pidX.setTolerance(kTranslationTolerance.in(Meters));
    m_pidY.setTolerance(kTranslationTolerance.in(Meters));
    addRequirements(swerveDrive);
    SmartDashboard.putData("ReefAlign/xPid", m_pidX);
    SmartDashboard.putData("ReefAlign/yPid", m_pidY);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pidX.reset(0.0);
    m_pidY.reset(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var leftPose = getTargetPose(m_cameraLeft, Meters.of(0.0001));
    var rightPose = getTargetPose(m_cameraRight, Meters.of(-0.0001));
    Logger.recordOutput("AlignReefTag/Pose", leftPose.isPresent() ? leftPose.get() : new Pose2d());
    var closestAprilTag = FieldPoint.getNearestPoint(m_swerveDrive.getPose(), FieldPoint.getReefAprilTagPoses());
    m_targetAngle = closestAprilTag.getCurrentAlliancePose().getRotation().getMeasure().plus(Degrees.of(180));

    if (leftPose.isEmpty() && rightPose.isEmpty()) {
      return;
    }
    
    double targetX = RobotDimensions.FrontBackLength.in(Meters) / 2 + RobotDimensions.RobotToReefMargin.in(Meters);
    double targetY = 0.0;
    Pose2d poseToUse = null;
    if (rightPose.isEmpty()) {
      // handle left camera, which most likely means right reef
      targetY = FieldDimensions.ReefBranchRight.getY();
      poseToUse = leftPose.get();
    } else if (leftPose.isEmpty()) {
      // handle right camera, which most likely means left reef
      targetY = FieldDimensions.ReefBranchLeft.getY();
      poseToUse = rightPose.get();
    } else {
      // figure out if left or right is closer
      double distanceLeft = leftPose.get().getTranslation().getNorm();
      double distanceRight = rightPose.get().getTranslation().getNorm();
      targetY = distanceLeft < distanceRight ? FieldDimensions.ReefBranchRight.getY() : FieldDimensions.ReefBranchLeft.getY();
      poseToUse = distanceLeft < distanceRight ? leftPose.get() : rightPose.get();
    }

    m_pidX.setGoal(new State(targetX, 0.0));
    m_pidY.setGoal(new State(targetY, 0.0));

    var speedX = MetersPerSecond.of(m_pidX.calculate(poseToUse.getX())).unaryMinus();
    var speedY = MetersPerSecond.of(m_pidY.calculate(poseToUse.getY())).unaryMinus();

    m_swerveDrive.moveRobotRelativeAngle(speedX, speedY, Rotation2d.fromDegrees(m_targetAngle.in(Degrees)), 1.0);
  }

  private Optional<Pose2d> getTargetPose(Camera camera, Distance simOffset) {
    if (Robot.isSimulation()) {
      var nearestPoint = FieldPoint.getNearestPoint(m_swerveDrive.getPose(), FieldPoint.getReefAprilTagPoses()).getCurrentAlliancePose();
      var robotRelativeTranslation = m_swerveDrive.getTranslatedPose(Meters.of(0.0), simOffset).relativeTo(nearestPoint);
      return Optional.of(robotRelativeTranslation);
    }
    Pose3d targetPose = camera.getTargetPose3dRobotSpace(); // Do we want the getBotPose3d_TargetSpace pose?
    boolean hasPose = targetPose.getX() == 0 && targetPose.getY() == 0;
    return hasPose ? Optional.of(targetPose.toPose2d()) : Optional.empty();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pidX.atGoal() && m_pidY.atGoal() && m_swerveDrive.getPose().getRotation().getMeasure().isNear(m_targetAngle, Degrees.of(3));
  }
}
