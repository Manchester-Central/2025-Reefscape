// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import frc.robot.subsystems.lift.IdLift.IdLiftValues;
import java.util.function.Supplier;

/** Add your docs here. */
public class Gripper extends AbstractLiftPart {
  private double m_targetSpeed = 0;

  public Gripper(Supplier<IdLiftValues> idLiftValuesSupplier) {
    super(idLiftValuesSupplier);
  }

  public void setTargetSpeed(double newSpeed) {
    m_targetSpeed = newSpeed;
  }

  public double getCurrentSpeed() {
    return m_targetSpeed;
  }
}
