// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.commands.SimpleDriveToPosition;
import frc.robot.subsystems.SwerveDrive;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Set;

/** Add your docs here. */
public class PathUtil {

  // Create the constraints to use while pathfinding
  static PathConstraints constraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  /**
   * Drives to the FieldPoint on the field (respective of the current alliance).
   */
  public static Command driveToPoseCommand(FieldPoint targetPostion, SwerveDrive swerveDrive) {
    return new DeferredCommand(
        () -> AutoBuilder.pathfindToPose(targetPostion.getCurrentAlliancePose(), constraints, 0.0),
        Set.of(swerveDrive));
  }

  /**
   * Drives to the closest FieldPoint on the field (respective of the current alliance).
   */
  public static Command driveToClosestPointAutoCommand(
      ArrayList<FieldPoint> possibleTargets, SwerveDrive swerveDrive, double timeOutSeconds) {
    return new DeferredCommand(
        () -> {
          ArrayList<Pose2d> possiblePoses = new ArrayList<Pose2d>();
          for (int i = 0; i < possibleTargets.size(); i++) {
            possiblePoses.add(possibleTargets.get(i).getCurrentAlliancePose());
          }
          Command simpleDriveToPosition = new SimpleDriveToPosition(swerveDrive, FieldPoint.getNearestPoint(swerveDrive.getPose(), possibleTargets));
          if (DriverStation.isAutonomousEnabled()) {
            simpleDriveToPosition = simpleDriveToPosition.withTimeout(timeOutSeconds);
          }
          return AutoBuilder.pathfindToPose(
              swerveDrive.getPose().nearest(possiblePoses), constraints, 0.0).andThen(simpleDriveToPosition);
        },
        Set.of(swerveDrive));
  }

  /**
   * Drives to the closest FieldPoint on the field (respective of the current alliance).
   */
  public static Command driveToClosestPointTeleopCommand(
      ArrayList<FieldPoint> possibleTargets, SwerveDrive swerveDrive) {
    return new DeferredCommand(
        () -> {
          ArrayList<Pose2d> possiblePoses = new ArrayList<Pose2d>();
          for (int i = 0; i < possibleTargets.size(); i++) {
            possiblePoses.add(possibleTargets.get(i).getCurrentAlliancePose());
          }
          Command simpleDriveToPosition = new SimpleDriveToPosition(swerveDrive, FieldPoint.getNearestPoint(swerveDrive.getPose(), possibleTargets));
          return AutoBuilder.pathfindToPose(
              swerveDrive.getPose().nearest(possiblePoses), constraints, 0.0).andThen(simpleDriveToPosition);
        },
        Set.of(swerveDrive));
  }
}
