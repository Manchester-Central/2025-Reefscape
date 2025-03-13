// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Constants.IoPortsConstants;
import frc.robot.Constants.ArmConstants.GripperConstants;
import frc.robot.subsystems.arm.Arm.ArmValues;
import frc.robot.Robot;
import frc.robot.utils.ChaosTalonFx;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Gripper extends AbstractArmPart {
  public static boolean hasCoralGrippedSim = false;

  private static boolean m_hasCoralGripped = false;

  private ChaosTalonFx m_coralMotor = new ChaosTalonFx(CanIdentifiers.GripperCoralMotorCANID);

  private DigitalInput m_coralSensor = new DigitalInput(IoPortsConstants.CoralChannelID);

  private Debouncer m_coralSensorDebouncer = new Debouncer(GripperConstants.CoralDropDebounceSeconds, DebounceType.kFalling);

  private ChaosTalonFx m_algaeMotor = new ChaosTalonFx(CanIdentifiers.GripperAlgaeMotorCANID);

  /**
   * Creates a new Gripper.
   *
   * @param armValuesSupplier the supplier of arm values
   */
  public Gripper(Supplier<ArmValues> armValuesSupplier) {
    super(armValuesSupplier);
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
   * Sets the speed [-1.0, 1.0] of the algae gripper.
   */
  public void setAlgaeGripSpeed(double newSpeed) {
    m_algaeMotor.set(newSpeed);
  }

  public double getAlgaeGripSpeed() {
    return m_algaeMotor.get();
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
    m_hasCoralGripped = m_coralSensorDebouncer.calculate(Robot.isSimulation() ? hasCoralGrippedSim : !m_coralSensor.get());
    Logger.recordOutput("Gripper/HasCoral", hasCoral());
  }
}
