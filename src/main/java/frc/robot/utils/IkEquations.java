package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.RobotDimensions;
import frc.robot.subsystems.lift.LiftPose;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

/**
 * Inverse Kinematic Equations for our 2025 Robot.
 */
public class IkEquations {
  public static Pose3d make3dFromIk2d(Pose2d point, Pose2d robotPose) {
    return new Pose3d(robotPose).transformBy(new Transform3d(point.getX(), 0, point.getY(), new Rotation3d(0, -1 * point.getRotation().getRadians(), 0)));
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
    Translation3d robotOriginToReefBranch = endEffectorPose.getTranslation().minus(robotPose.getTranslation());
    Distance floorDistance = Meters.of(robotOriginToReefBranch.toTranslation2d().getNorm());
    // Calculates the max range of the robot's reach.
    // This doesn't account for negative end effector reaching, but that's not a real problem we have to deal with.
    Distance mechanismExtensionRange = Meters.of(RobotDimensions.FrontBackLengthMetersFrame / 2)
                                      .plus(RobotDimensions.MechanismExtensionMargin);
    // Now take the min of the 2 distances
    floorDistance = mechanismExtensionRange.lt(floorDistance) ? mechanismExtensionRange : floorDistance;
    Logger.recordOutput("IkSolver/FloorDistance", floorDistance);
    Pose2d constrainedEndEffectorPoint = new Pose2d(floorDistance, Meters.of(endEffectorPose.getZ()),
                                                    Rotation2d.fromRadians(-1.0 * endEffectorPose.getRotation().getY()));
    Pose3d constrainedEndEffectorPose = make3dFromIk2d(constrainedEndEffectorPoint, robotPose.toPose2d());
    Logger.recordOutput("IkSolver/ConstrainedEndEffectorPoint", constrainedEndEffectorPoint);
    Logger.recordOutput("IkSolver/ConstrainedEndEffectorPose", constrainedEndEffectorPose);

    // Calculate End Effector Wrist Point
    Pose2d endEffectorWristPoint = constrainedEndEffectorPoint.transformBy(new Transform2d(
        -(RobotDimensions.WristToCoralTip.getTranslation().getNorm() + RobotDimensions.CoralPlacementMargin), 0, Rotation2d.kZero));
    Pose3d endEffectorWristPose = make3dFromIk2d(endEffectorWristPoint, robotPose.toPose2d());
    Logger.recordOutput("IkSolver/EndEffectorWristPose", endEffectorWristPose);

    /**
     * This is the simple solution, it should put the coral where it wants to be within 1.5 cm.
     */

    // // Calculate the middle space between the wrist and the base pivot
    // Translation2d hvector = endEffectorWristPoint.getTranslation()
    //                         .minus(RobotDimensions.BasePivotOffset.getTranslation());
    // Logger.recordOutput("IkSolver/hvector_length", hvector.getNorm());
    // var hlength = hvector.getNorm() - RobotDimensions.LiftToWristOffset.getTranslation().getNorm();
    // Logger.recordOutput("IkSolver/hlength", hlength);
    // // Naive angle assuming our Carriage to GripperPivot is along a line... which it almost is...
    // var hangle = hvector.getAngle();
    // var gripperPivotAngle = hangle.times(-1)
    //                         .plus(Rotation2d.fromRadians(-endEffectorPose.getRotation().getY()))
    //                         .plus(RobotDimensions.WristMountAngle);
    // return new LiftPose("IkCalculatedPose", hangle.getDegrees(), hlength, gripperPivotAngle.getDegrees());

    /*
     * This is the more accurate IK solve
     */

    // Get core problem dimensions
    Translation2d hvector = endEffectorWristPoint.getTranslation().minus(RobotDimensions.BasePivotOffset.getTranslation());
    Distance hlength = Meters.of(hvector.getNorm());
    Logger.recordOutput("IkSolver/hvector_length", hlength);
    Rotation2d hangle = hvector.getAngle(); // Mechanism Coord Frame, so positive is up
    Distance clength = Meters.of(RobotDimensions.LiftToWristOffset.getTranslation().getNorm());

    var angleB = RobotDimensions.WristMountAngle;
    var alength = clength.in(Meters) * Math.sin(angleB.getRadians());
    var blength = clength.in(Meters) * Math.cos(angleB.getRadians());
    var omegaRadians = Rotation2d.fromRadians(Math.asin(alength / hlength.in(Meters)));
    var blLength = hlength.in(Meters) * Math.cos(omegaRadians.getRadians());

    // Finally, the 2 values we care about:
    // - The Base Pivot angle (alpha)
    // - Extender Length (lLength)
    Rotation2d alpha = hangle.plus(omegaRadians);
    double liftLength = blLength - blength;
    Rotation2d gripperPivotAngle = alpha.times(-1)
                                   .plus(Rotation2d.fromRadians(-endEffectorPose.getRotation().getY()));
    // Examples:
    // Lift Angle (alpha) degrees, desired gripper angle -> effective gripper angle relative to lift
    // Lift 45d, gripper pivot 0d -> gripper -(45)+(0) = -45d
    // Lift 0d, gripper pivot 0d -> gripper -(0)+(0) = 0d
    // Lift 60d, gripper pivot -60d -> gripper -(60)+(-60) = -120d
    // Lift 90d, gripper pivot -45d -> gripper -(90)+(-45) = -135d

    return new LiftPose("IkCalculatedPose", alpha, liftLength, gripperPivotAngle);
  }

  public static LiftPose getPivotLiftPivot(Pose2d robotPose, Pose3d endEffectorPose) {
    return getPivotLiftPivot(new Pose3d(robotPose), endEffectorPose);
  }
}
