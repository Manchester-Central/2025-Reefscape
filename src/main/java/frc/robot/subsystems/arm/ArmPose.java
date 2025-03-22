// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.chaos131.util.DashboardNumber;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Optional;

/** A class for maintain goal points for certain arm poses. */
public class ArmPose {
  private String m_name;
  private Rotation2d m_basePivotAngle;
  private double m_extensionMeters;
  private Rotation2d m_gripperPivotAngle;
  private Optional<Rotation2d> m_basePivotSafetyAngle = Optional.empty();

  /** Creates a new arm pose. */
  public ArmPose(String name, double basePivotDegrees, double extensionMeters, double gripperPivotDegrees) {
    this(name, Rotation2d.fromDegrees(basePivotDegrees), extensionMeters, Rotation2d.fromDegrees(gripperPivotDegrees));
  }

  /** Creates a new arm pose. */
  public ArmPose(String name, Rotation2d basePivotAngle, double extensionMeters, Rotation2d gripperPivotAngle) {
    m_name = name;
    m_basePivotAngle = basePivotAngle;
    m_extensionMeters = extensionMeters;
    m_gripperPivotAngle = gripperPivotAngle;

    new DashboardNumber(
      "Arm/Pose/" + m_name + "/Base Pivot Degrees", basePivotAngle.getDegrees(), true, newDegrees -> m_basePivotAngle = Rotation2d.fromDegrees(newDegrees)
    );
    new DashboardNumber(
      "Arm/Pose/" + m_name + "/Extension Meters", extensionMeters, true, newMeters -> m_extensionMeters = newMeters
    );
    new DashboardNumber(
      "Arm/Pose/" + m_name + "/Gripper Pivot Degrees", gripperPivotAngle.getDegrees(), true, newDegrees -> m_gripperPivotAngle = Rotation2d.fromDegrees(newDegrees)
    );
  }

  /**
   * add an angle that the base pivot will go to before the proper target.
   */
  public ArmPose withBasePivotSafety(Rotation2d safetyRotation) {
    m_basePivotSafetyAngle = Optional.of(safetyRotation);
    return this;
  }
  
  /**
   * add an angle that the base pivot will go to before the proper target.
   */
  public ArmPose withBasePivotSafety(double safetyRotation) {
    return withBasePivotSafety(Rotation2d.fromDegrees(safetyRotation));
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

  public Optional<Rotation2d> getBasePivotSafetyAngle() {
    return m_basePivotSafetyAngle;
  }
}
