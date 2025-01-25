// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

/** Add your docs here. */
public class Extender {
  private double m_targetPosition;

  public void setTargetPosition(double newPosition) {
    m_targetPosition = newPosition;
  }

  public double getCurrentPosition() {
    return m_targetPosition;
  }
}
