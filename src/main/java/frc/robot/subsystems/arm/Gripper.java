// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;

import com.chaos131.util.DashboardNumber;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ArmConstants.GripperConstants;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Constants.IoPortsConstants;
import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm.ArmValues;
import frc.robot.utils.ChaosTalonFx;
import frc.robot.utils.ChaosTalonFxTuner;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Gripper extends AbstractArmPart {
  public static boolean hasCoralGrippedSim = false;

  private static boolean m_hasCoralGripped = false;

  public static boolean hasAlgaeGrippedSim = false;

  private static boolean m_hasAlgaeGripped = false;


  private ChaosTalonFx m_coralMotor = new ChaosTalonFx(CanIdentifiers.GripperCoralMotorCANID);

  private DigitalInput m_coralSensor = new DigitalInput(IoPortsConstants.CoralChannelID);

  private Debouncer m_coralSensorDebouncer = new Debouncer(GripperConstants.CoralDropDebounceSeconds, DebounceType.kFalling);

  private ChaosTalonFx m_algaeMotor = new ChaosTalonFx(CanIdentifiers.GripperAlgaeMotorCANID);

  private Debouncer m_algaeSensorDebouncer = new Debouncer(GripperConstants.AlgaeDropDebounceSeconds, DebounceType.kFalling);

  private ChaosTalonFxTuner m_algaeTuner = new ChaosTalonFxTuner("AlgaeGripper", m_algaeMotor);
  // Current limits
  private DashboardNumber m_algaeSupplyCurrentLimit = m_algaeTuner.tunable(
      "SupplyCurrentLimit", GripperConstants.AlgaeSupplyCurrentLimit.in(Amps), (config, newValue) -> config.CurrentLimits.SupplyCurrentLimit = newValue);
  private DashboardNumber m_algaeStatorCurrentLimit = m_algaeTuner.tunable(
      "StatorCurrentLimit", GripperConstants.AlgaeStatorCurrentLimit.in(Amps), (config, newValue) -> config.CurrentLimits.StatorCurrentLimit = newValue);

  /**
   * Creates a new Gripper.
   *
   * @param armValuesSupplier the supplier of arm values
   */
  public Gripper(Supplier<ArmValues> armValuesSupplier) {
    super(armValuesSupplier);
    m_algaeMotor.Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    m_algaeMotor.Configuration.CurrentLimits.StatorCurrentLimit = m_algaeStatorCurrentLimit.get();
    m_algaeMotor.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_algaeMotor.Configuration.CurrentLimits.SupplyCurrentLimit = m_algaeSupplyCurrentLimit.get();
    m_algaeMotor.applyConfig();
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

  /**
   * Checks if we have Algae.
   */
  public boolean hasAlgae() {
    return m_hasAlgaeGripped;
  }


  @Override
  public void periodic() {
    super.periodic();
    m_hasCoralGripped = m_coralSensorDebouncer.calculate(Robot.isSimulation() ? hasCoralGrippedSim : !m_coralSensor.get());
    boolean algaeCurrentLimitReached = m_algaeMotor.getStatorCurrent().getValue().gt(Amps.of(m_algaeStatorCurrentLimit.get() - 0.1));
    m_hasAlgaeGripped = m_algaeSensorDebouncer.calculate(Robot.isSimulation() ? hasAlgaeGrippedSim : algaeCurrentLimitReached);
    Logger.recordOutput("Gripper/HasCoral", hasCoral());
    Logger.recordOutput("Gripper/HasAlgae", hasAlgae());
  }
}
