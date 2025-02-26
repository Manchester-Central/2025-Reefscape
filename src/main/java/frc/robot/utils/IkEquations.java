package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import frc.robot.Constants.RobotDimensions;
import frc.robot.subsystems.lift.LiftPose;
import org.littletonrobotics.junction.Logger;

/**
 * Inverse Kinematic Equations for our 2025 Robot.
 */
public class IkEquations {
  public static Pose3d make3dFromIk2d(Translation2d point, Pose2d robotPose) {
    return new Pose3d(robotPose).transformBy(new Transform3d(point.getX(), 0, point.getY(), Rotation3d.kZero));
  }

  /**
   * Converts the 3d points into a mechanism 2d coordinate frame, typically start point is the robot pose,
   * or some lower part on the robot. Only generates vectors with a positive X value.
   *
   * @param startPoint - origin of the vector, typically robot pose
   * @param targetPoint - point to point at, normally a joint or end location
   * @return
   */
  public static Translation2d makeMechanismPointFrom3dPoses(Translation3d startPoint, Translation3d targetPoint) {
    var diff = targetPoint.minus(startPoint);
    return new Translation2d(diff.toTranslation2d().getNorm(), diff.getZ());
  }

  /**
   * Calculates the Pivots and Extender Lengths in order to reach a specific pose.
   *
   * @param robotPose of the robot origin (either robot coordinate frame or field coordinates)
   * @param endEffectorPose of the END of the end effector, not the start of it (must be consistent with the coordinate frame)
   * @return The final pose, should never be null
   */
  public static LiftPose getPivotLiftPivot(Pose3d robotPose, Pose3d endEffectorPose) {
    Logger.recordOutput("IkSolver/EndEffectorPose", endEffectorPose);
    // Robot Origin to Reef Branch
    var robotOriginToReefBranch = endEffectorPose.getTranslation().minus(robotPose.getTranslation());
    var floorDistance = robotOriginToReefBranch.toTranslation2d().getNorm();
    // Calculates the max range of the robot's reach.
    // This doesn't account for negative end effector reaching, but that's not a real problem we have to deal with.
    double mechanismExtensionRange = (RobotDimensions.FrontBackLengthMetersFrame / 2) + RobotDimensions.MechanismExtensionMargin.in(Units.Meters);
    // Now take the min of the 2 distances
    floorDistance = mechanismExtensionRange < floorDistance ? mechanismExtensionRange : floorDistance;
    Logger.recordOutput("IkSolver/FloorDistance", floorDistance);
    var constrainedEndEffectorPoint = make3dFromIk2d(new Translation2d(floorDistance, endEffectorPose.getZ()), robotPose.toPose2d());
    constrainedEndEffectorPoint = constrainedEndEffectorPoint.transformBy(new Transform3d(new Translation3d(), endEffectorPose.getRotation()));
    Logger.recordOutput("IkSolver/EffectiveEndEffectorPoint", constrainedEndEffectorPoint);

    // Calculate End Effector Wrist Point
    var endEffectorWrist = constrainedEndEffectorPoint.transformBy(new Transform3d(
        -RobotDimensions.WristToEndEffector.getTranslation().getNorm(), 0, 0, Rotation3d.kZero));
    Logger.recordOutput("IkSolver/EndEffectorWrist", endEffectorWrist.getTranslation());

    // Calculate Mechanism Root in 2d mechanism space
    var basePivotPoint = /* (0,0).plus */RobotDimensions.BasePivotOffset.getTranslation();

    var endEffectorWristMechanismSpace = makeMechanismPointFrom3dPoses(robotPose.getTranslation(), endEffectorWrist.getTranslation());

    var hvector = endEffectorWristMechanismSpace.minus(basePivotPoint);
    var hlength = hvector.getNorm() - RobotDimensions.LiftToWristOffset.getTranslation().getNorm();
    var hangle = hvector.getAngle();
    var gripperPivotAngle = hangle.times(-1).minus(Rotation2d.fromRadians(endEffectorPose.getRotation().getY()));

    // // Get core problem dimensions
    // var hvector = gripperPivotPoint.getTranslation().minus(basePivotPoint);
    // var hlength = hvector.getNorm();
    // var hangle = hvector.getAngle();
    // var clength = RobotDimensions.LiftToWristOffset.getTranslation().getNorm();

    // var angleB = Rotation2d.fromDegrees(180.0).minus(RobotDimensions.WristMountAngle);
    // var alength = clength * Math.sin(angleB.getRadians());
    // var blength = clength * Math.cos(angleB.getRadians());
    // var omegaRadians = Rotation2d.fromRadians(Math.asin(alength / hlength));
    // var blLength = hlength * Math.cos(omegaRadians.getRadians());

    // // Finally, the 2 values we care about:
    // // - The Base Pivot angle (alpha)
    // // - Extender Length (lLength)
    // var alpha = hangle.plus(omegaRadians);
    // var liftLength = blLength - blength;
    // var gripperPivotAngle = alpha.times(-1).plus(Rotation2d.fromRadians(endEffectorPose.getRotation().getY()));
    // Examples:
    // Lift Angle (alpha) degrees, desired gripper angle -> effective gripper angle relative to lift
    // Lift 45d, gripper pivot 0d -> gripper -(45)+(0) = -45d
    // Lift 0d, gripper pivot 0d -> gripper -(0)+(0) = 0d
    // Lift 60d, gripper pivot -60d -> gripper -(60)+(-60) = -120d
    // Lift 90d, gripper pivot -45d -> gripper -(90)+(-45) = -135d

    return new LiftPose("IkCalculatedPose", hangle, hlength, gripperPivotAngle);
  }

  public static LiftPose getPivotLiftPivot(Pose2d robotPose, Pose3d endEffectorPose) {
    return getPivotLiftPivot(new Pose3d(robotPose), endEffectorPose);
  }
}
