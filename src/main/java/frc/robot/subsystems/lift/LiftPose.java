// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import com.chaos131.util.DashboardNumber;
import edu.wpi.first.math.geometry.Rotation2d;

/** A class for maintain goal points for certain lift poses. */
public class LiftPose {
  private String m_name;
  private Rotation2d m_basePivotAngle;
  private double m_extensionMeters;
  private Rotation2d m_gripperPivotAngle;

  /** Creates a new lift pose. */
  public LiftPose(String name, double basePivotDegrees, double extensionMeters, double gripperPivotDegrees) {
    this(name, Rotation2d.fromDegrees(basePivotDegrees), extensionMeters, Rotation2d.fromDegrees(gripperPivotDegrees));
  }

  /** Creates a new lift pose. */
  public LiftPose(String name, Rotation2d basePivotAngle, double extensionMeters, Rotation2d gripperPivotAngle) {
    m_name = name;
    m_basePivotAngle = basePivotAngle;
    m_extensionMeters = extensionMeters;
    m_gripperPivotAngle = gripperPivotAngle;

    new DashboardNumber(
      "Lift/Pose/" + m_name + "/Base Pivot Degrees", basePivotAngle.getDegrees(), true, newDegrees -> m_basePivotAngle = Rotation2d.fromDegrees(newDegrees)
    );
    new DashboardNumber(
      "Lift/Pose/" + m_name + "/Extension Meters", extensionMeters, true, newMeters -> m_extensionMeters = newMeters
    );
    new DashboardNumber(
      "Lift/Pose/" + m_name + "/Gripper Pivot Degrees", gripperPivotAngle.getDegrees(), true, newDegrees -> m_gripperPivotAngle = Rotation2d.fromDegrees(newDegrees)
    );
  }

  public Rotation2d getBasePivotAngle() {
    return m_basePivotAngle;
  }

  public double getExtensionMeters() {
    return m_extensionMeters;
  }

  public Rotation2d getGripperPivotAngle() {
    return m_gripperPivotAngle;
  }
}
