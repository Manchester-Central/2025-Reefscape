// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Constants.IoPortsConstants;
import frc.robot.Robot;
import frc.robot.subsystems.lift.IdLift.IdLiftValues;
import frc.robot.utils.ChaosTalonFx;
import java.util.function.Supplier;

/** Add your docs here. */
public class Gripper extends AbstractLiftPart {
  private double m_targetSpeed = 0;
  private double m_gearRation = 40.0;
  private double m_jkgMetersSquared = 0.1;
  public static boolean hasCoralFrontGrippedSim = false;
  public static boolean hasCoralBackGrippedSim = false;
  public static boolean hasAlgaeGrippedSim = false;
  private DCMotor m_dcMotor = DCMotor.getKrakenX60(1);
  private DCMotorSim m_motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(m_dcMotor, m_jkgMetersSquared, m_gearRation),
          m_dcMotor,
          0.001,
          0.001);

  private ChaosTalonFx m_motor =
      new ChaosTalonFx(CanIdentifiers.GripperMotorCANID, m_gearRation, m_motorSim, true);

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

  public void setTargetSpeed(double newSpeed) {
    m_targetSpeed = newSpeed;
  }

  public double getCurrentSpeed() {
    return m_targetSpeed;
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
