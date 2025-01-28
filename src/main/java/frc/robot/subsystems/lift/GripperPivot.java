// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class GripperPivot extends SubsystemBase {
  private Rotation2d m_targetAngle = Rotation2d.fromDegrees(0);

  public void setTargetAngle(Rotation2d newAngle) {
    m_targetAngle = newAngle;
  }

  public Rotation2d getCurrentAngle() {
    return m_targetAngle; // TODO get actual motor angle
  }
}
