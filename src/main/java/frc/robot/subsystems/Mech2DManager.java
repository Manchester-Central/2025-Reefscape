// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Mech2DManager {
  // @AutoLogOutput(key = "Manipulator")
  private Mechanism2d m_mechanismBase;
  private MechanismRoot2d m_mechanismRoot;

  public Mech2DManager() {
    m_mechanismBase = new Mechanism2d(0.889, 3);
    m_mechanismRoot = m_mechanismBase.getRoot("Manipulator", 0, 0);
    SmartDashboard.putData("Manipulator", m_mechanismBase);
  }

  public MechanismRoot2d getMechRoot() {
    return m_mechanismRoot;
  }
}
