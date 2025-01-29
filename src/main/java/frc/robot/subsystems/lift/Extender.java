// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Extender extends SubsystemBase {
  private double m_targetLength = 1;

  public void setTargetLength(double newLength) {
    m_targetLength = newLength;
  }

  public double getCurrentLength() {
    return m_targetLength;
  }
}
