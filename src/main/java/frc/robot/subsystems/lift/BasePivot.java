// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class BasePivot {
  private Rotation2d m_targetAngle = Rotation2d.fromDegrees(120);

  public void setTargetAngle(Rotation2d newAngle) {
    m_targetAngle = newAngle;
  }

  public Rotation2d getCurrentAngle() {
    return m_targetAngle; // TODO get actual motor angle
  }
}
