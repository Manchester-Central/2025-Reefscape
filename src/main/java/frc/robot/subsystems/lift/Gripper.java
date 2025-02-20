// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Constants.IoPortsConstants;
import frc.robot.Constants.MidLiftConstants.GripperConstants;
import frc.robot.Robot;
import frc.robot.subsystems.lift.IdLift.IdLiftValues;
import frc.robot.utils.ChaosTalonFx;
import java.util.function.Supplier;

/** Add your docs here. */
public class Gripper extends AbstractLiftPart {
  public static boolean hasCoralGrippedSim = false;

  private static boolean m_hasCoralGripped = false;

  private ChaosTalonFx m_coralMotor = new ChaosTalonFx(CanIdentifiers.GripperCoralMotorCANID);

  private DigitalInput m_coralSensor = new DigitalInput(IoPortsConstants.CoralChannelID);

  private Debouncer m_coralSensorDebouncer = new Debouncer(GripperConstants.CoralDropDebounceSeconds, DebounceType.kFalling);

  /**
   * Creates a new Gripper.
   *
   * @param idLiftValuesSupplier the supplier of lift values
   */
  public Gripper(Supplier<IdLiftValues> idLiftValuesSupplier) {
    super(idLiftValuesSupplier);
  }

  /**
   * Sets the speed [-1.0, 1.0] of the coral gripper.
   */
  public void setCoralGripSpeed(double newSpeed) {
    m_coralMotor.set(newSpeed);
  }

  public double getCoralGripSpeed() {
    return m_coralMotor.get();
  }

  /**
   * Checks if there is a coral at the sensor.
   */
  public boolean hasCoral() {
    return m_hasCoralGripped;
  }


  @Override
  public void periodic() {
    super.periodic();
    m_hasCoralGripped = m_coralSensorDebouncer.calculate(Robot.isSimulation() ? hasCoralGrippedSim : m_coralSensor.get());
  }
}
