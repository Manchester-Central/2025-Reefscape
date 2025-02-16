// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.lift.IdLift.IdLiftValues;
import java.util.function.Supplier;

/** A class that stores values/functions that any lift part should have. */
public abstract class AbstractLiftPart extends SubsystemBase {
  private Supplier<IdLiftValues> m_idLiftValuesSupplier;

  /**
   * The base constructor for any lift part.
   *
   * @param idLiftValuesSupplier the supplier of lift values
   */
  protected AbstractLiftPart(Supplier<IdLiftValues> idLiftValuesSupplier) {
    m_idLiftValuesSupplier = idLiftValuesSupplier;
  }

  protected IdLiftValues getLiftValues() {
    return m_idLiftValuesSupplier.get();
  }
}
