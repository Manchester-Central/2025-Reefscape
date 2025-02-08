// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class PathUtil {

  // Create the constraints to use while pathfinding
  static PathConstraints constraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  public static Command toCreateFindAPathCommand(Pose2d targetPostion) {
    Command pathfindingCommand = AutoBuilder.pathfindToPose(targetPostion, constraints, 0.0);
    return pathfindingCommand;
  }
}
