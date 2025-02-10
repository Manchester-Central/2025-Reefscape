// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Constants.IoPortsConstants;
import frc.robot.Robot;
import frc.robot.subsystems.lift.IdLift.IdLiftValues;
import frc.robot.utils.ChaosTalonFx;
import java.util.function.Supplier;

/** Add your docs here. */
public class Gripper extends AbstractLiftPart {
  public static boolean hasCoralFrontGrippedSim = false;
  public static boolean hasCoralBackGrippedSim = false;
  public static boolean hasAlgaeGrippedSim = false;

  private ChaosTalonFx m_coralMotor = new ChaosTalonFx(CanIdentifiers.GripperCoralMotorCANID);
  private ChaosTalonFx m_algaeMotor = new ChaosTalonFx(CanIdentifiers.GripperAlgaeMotorCANID);

  private DigitalInput m_algaeSensor = new DigitalInput(IoPortsConstants.AlgaeChannelID);
  private DigitalInput m_coralSensor1 = new DigitalInput(IoPortsConstants.CoralOneChannelID);
  private DigitalInput m_coralSensor2 = new DigitalInput(IoPortsConstants.CoralTwoChannelID);

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

  /**
   * Sets the speed [-1.0, 1.0] of the algae gripper.
   */
  public void setAlgaeGripSpeed(double newSpeed) {
    m_algaeMotor.set(newSpeed);
  }

  public double getCoralGripSpeed() {
    return m_coralMotor.get();
  }

  public double getAlgaeGripSpeed() {
    return m_algaeMotor.get();
  }

  /**
   * Checks if there is a coral at the front sensor.
   */
  public boolean hasCoralFront() {
    if (Robot.isSimulation()) {
      return hasCoralFrontGrippedSim;
    }
    return m_coralSensor1.get();
  }

  /**
   * Checks if there is a coral at the back sensor.
   */
  public boolean hasCoralBack() {
    if (Robot.isSimulation()) {
      return hasCoralBackGrippedSim;
    }
    return m_coralSensor2.get();
  }

  /**
   * Checks if there is an alage at the sensor.
   */
  public boolean hasAlgae() {
    if (Robot.isSimulation()) {
      return hasAlgaeGrippedSim;
    }
    return m_algaeSensor.get();
  }
}
