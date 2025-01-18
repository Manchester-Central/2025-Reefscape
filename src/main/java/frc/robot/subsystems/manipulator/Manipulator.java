// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Lift;

/** Add your docs here. */
public class Manipulator {
  // @AutoLogOutput(key = "Manipulator")
  private Mechanism2d m_mechanismBase;
  private MechanismRoot2d m_mechanismRoot;
  public Lift m_lift;

  public Manipulator() {
    m_mechanismBase = new Mechanism2d(3, 3);
    m_mechanismRoot = m_mechanismBase.getRoot("Manipulator", 0, 0);
    m_lift = new Lift(m_mechanismRoot);
    SmartDashboard.putData("Manipulator", m_mechanismBase);
  }
}
