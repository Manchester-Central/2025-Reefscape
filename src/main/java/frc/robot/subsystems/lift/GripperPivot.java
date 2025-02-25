// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Constants.MidLiftConstants.GripperPivotConstants;
import frc.robot.Constants.MidLiftConstants.LiftPoses;
import frc.robot.subsystems.lift.IdLift.IdLiftValues;
import frc.robot.utils.ChaosCanCoder;
import frc.robot.utils.ChaosCanCoderTuner;
import frc.robot.utils.ChaosTalonFx;
import frc.robot.utils.ChaosTalonFxTuner;

import java.util.Currency;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class GripperPivot extends AbstractLiftPart {
  private double m_simGearRatio = GripperPivotConstants.RotorToSensorRatio;
  private double m_jkgMetersSquared = 0.1;
  private Rotation2d m_targetAngle = Rotation2d.fromDegrees(120);
  private DCMotor m_dcMotor = DCMotor.getKrakenX60(1);
  private DCMotorSim m_motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(m_dcMotor, m_jkgMetersSquared, m_simGearRatio),
          m_dcMotor,
          0.001,
          0.001);
  private ChaosTalonFx m_motor =
      new ChaosTalonFx(CanIdentifiers.GripperPivotMotorCANID);
  private ChaosCanCoder m_canCoder =
      new ChaosCanCoder(CanIdentifiers.GripperPivotCANCoderCANID);

  private ChaosTalonFxTuner m_tuner = new ChaosTalonFxTuner("GripperPivot", m_motor);
  private ChaosCanCoderTuner m_canCoderTuner = new ChaosCanCoderTuner("Gripper Pivot", m_canCoder);

  private DashboardNumber m_canCoderOffsetDegrees = m_canCoderTuner.tunable("CANCoder Tuner",
      GripperPivotConstants.canCoderOffsetDegrees, (config, newValue) -> 
      config.MagnetSensor.MagnetOffset = Rotation2d.fromDegrees(newValue).getRotations());

  // Motion Magic Slot 0 Configs
  private DashboardNumber m_kp = m_tuner.tunable("kP", GripperPivotConstants.kP, (config, newValue) -> config.Slot0.kP = newValue);
  private DashboardNumber m_ki = m_tuner.tunable("kI", GripperPivotConstants.kI, (config, newValue) -> config.Slot0.kI = newValue);
  private DashboardNumber m_kd = m_tuner.tunable("kD", GripperPivotConstants.kD, (config, newValue) -> config.Slot0.kD = newValue);
  private DashboardNumber m_kg = m_tuner.tunable("kG", GripperPivotConstants.kG, (config, newValue) -> config.Slot0.kG = newValue);
  private DashboardNumber m_ks = m_tuner.tunable("kS", GripperPivotConstants.kS, (config, newValue) -> config.Slot0.kS = newValue);
  private DashboardNumber m_kv = m_tuner.tunable("kV", GripperPivotConstants.kV, (config, newValue) -> config.Slot0.kV = newValue);
  private DashboardNumber m_ka = m_tuner.tunable("kA", GripperPivotConstants.kA, (config, newValue) -> config.Slot0.kA = newValue);

  // Motion Magic Constraints
  private DashboardNumber m_mmCruiseVelocity = m_tuner.tunable(
      "MM_CruiseVelocity", GripperPivotConstants.MMCruiseVelocity, (config, newValue) -> config.MotionMagic.MotionMagicCruiseVelocity = newValue);
  private DashboardNumber m_mmAcceleration = m_tuner.tunable(
      "MM_Acceleration", GripperPivotConstants.MMAcceleration, (config, newValue) -> config.MotionMagic.MotionMagicAcceleration = newValue);
  private DashboardNumber m_mmJerk = m_tuner.tunable(
      "MM_Jerk", GripperPivotConstants.MMJerk, (config, newValue) -> config.MotionMagic.MotionMagicJerk = newValue);

  // Current limits
  private DashboardNumber m_supplyCurrentLimit = m_tuner.tunable(
      "SupplyCurrentLimit", GripperPivotConstants.SupplyCurrentLimit, (config, newValue) -> config.CurrentLimits.SupplyCurrentLimit = newValue);
  private DashboardNumber m_statorCurrentLimit = m_tuner.tunable(
      "StatorCurrentLimit", GripperPivotConstants.StatorCurrentLimit, (config, newValue) -> config.CurrentLimits.StatorCurrentLimit = newValue);

  // Sensor Feedback
  private DashboardNumber m_rotorToSensorRatio = m_tuner.tunable("RotorToSensorRatio", GripperPivotConstants.RotorToSensorRatio,
      (config, newValue) -> config.Feedback.RotorToSensorRatio = newValue);
  private DashboardNumber m_sensorToMechRatio = m_tuner.tunable("SensorToMechanismRatio", GripperPivotConstants.SensorToMechanismRatio,
      (config, newValue) -> config.Feedback.SensorToMechanismRatio = newValue);

  // Ramp rates
  private DashboardNumber m_rampPeriod = m_tuner.tunable("VoltageClosedLoopRampPeriod", GripperPivotConstants.VoltageClosedLoopRampPeriod,
      (config, newValue) -> config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = newValue);

  /**
   * Creates a new GripperPivot.
   *
   * @param idLiftValuesSupplier the supplier of lift values
   */
  public GripperPivot(Supplier<IdLiftValues> idLiftValuesSupplier) {
    super(idLiftValuesSupplier);
    m_canCoder.Configuration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    m_canCoder.Configuration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    m_canCoder.Configuration.MagnetSensor.MagnetOffset = Rotation2d.fromDegrees(m_canCoderOffsetDegrees.get()).getRotations();
    m_canCoder.applyConfig();

    m_motor.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_motor.Configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_motor.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_motor.Configuration.CurrentLimits.SupplyCurrentLimit = m_supplyCurrentLimit.get();
    m_motor.Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    m_motor.Configuration.CurrentLimits.StatorCurrentLimit = m_statorCurrentLimit.get();
    m_motor.Configuration.Feedback.FeedbackRemoteSensorID = CanIdentifiers.GripperPivotCANCoderCANID;
    m_motor.Configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    // m_motor.Configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    m_motor.Configuration.Feedback.RotorToSensorRatio = m_rotorToSensorRatio.get();
    m_motor.Configuration.Feedback.SensorToMechanismRatio = m_sensorToMechRatio.get();
    m_motor.Configuration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = m_rampPeriod.get();
    m_motor.Configuration.MotionMagic.MotionMagicCruiseVelocity = m_mmCruiseVelocity.get();
    m_motor.Configuration.MotionMagic.MotionMagicAcceleration = m_mmAcceleration.get();
    m_motor.Configuration.MotionMagic.MotionMagicJerk = m_mmJerk.get();

    var slot0 = new Slot0Configs();
    slot0.kP = m_kp.get();
    slot0.kI = m_ki.get();
    slot0.kD = m_kd.get();
    slot0.kG = m_kg.get();
    slot0.kS = m_ks.get();
    slot0.kV = m_kv.get();
    slot0.kA = m_ka.get();
    m_motor.Configuration.Slot0 = slot0;

    m_motor.applyConfig();

    m_motor.attachMotorSim(m_motorSim, m_simGearRatio, ChassisReference.Clockwise_Positive, true);
    m_motor.attachCanCoderSim(m_canCoder);
  }

  /**
   * Sets the target angle and tries to drive there.
   */
  public void setTargetAngle(Rotation2d newAngle) {
    if (newAngle.getDegrees() > GripperPivotConstants.MaxAngle.getDegrees()) {
      newAngle = GripperPivotConstants.MaxAngle;
    } else if (newAngle.getDegrees() < GripperPivotConstants.MinAngle.getDegrees()) {
      newAngle = GripperPivotConstants.MinAngle;
    }

    // if (!getLiftValues().isBasePivotAtSafeAngle || !getLiftValues().isExtenderAtSafeLength) {
    //   newAngle = LiftPoses.Stow.getGripperPivotAngle();
    // }
    m_targetAngle = newAngle;
    m_motor.moveToPositionMotionMagic(newAngle.getRotations());
  }

  public boolean isSafeAngle() {
    return Math.abs(getCurrentAngle().minus(GripperPivotConstants.SafeAngle).getDegrees()) < GripperPivotConstants.SafeAngleTolerance.getDegrees();
  }

  /**
   * Sets the direct speed [-1.0, 1.0] of the motors.
   */
  public void setSpeed(double speed) {
    if (getCurrentAngle().getDegrees() > GripperPivotConstants.MaxAngle.getDegrees()) {
      speed = Math.min(speed, 0.0);
    } else if (getCurrentAngle().getDegrees() < GripperPivotConstants.MinAngle.getDegrees()) {
      speed = Math.max(speed, 0.0);
    }
    m_motor.set(speed);
  }

  public double getSpeed() {
    return m_motor.getVelocity().getValueAsDouble();
  }

  public Rotation2d getCurrentAngle() {
    return Rotation2d.fromRotations(
        m_canCoder.getAbsolutePosition().getValueAsDouble());
  }

  /**
   * Checks if the current angle is at the goal angle.
   */
  public boolean atTarget() {
    return Math.abs(getCurrentAngle().minus(m_targetAngle).getDegrees()) < 0.1;
  }

  @Override
  public void simulationPeriodic() {
    m_motor.simUpdate();
  }

  /**
   * Set extender motor to Coast. :3
   */ 
  public void setMotorCoast() {
    m_motor.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_motor.applyConfig();
  }

  /**
   * Set extender motor to Brake. :3
   */ 
  public void setMotorBrake() {
    m_motor.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_motor.applyConfig();
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    super.periodic();
    Logger.recordOutput("GripperPivot/Setpoint", m_targetAngle);
    Logger.recordOutput("GripperPivot/CurrentAngle", getCurrentAngle().getDegrees());
    Logger.recordOutput("GripperPivot/AtTarget", atTarget());
    Logger.recordOutput("GripperPivot/AngleError", getCurrentAngle().minus(m_targetAngle));
    Logger.recordOutput("GripperPivot/Voltage", m_motor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("GripperPivot/StatorCurrent", m_motor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("GripperPivot/SupplyCurrent", m_motor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("GripperPivot/MotorAngle", Rotation2d.fromRotations(m_motor.getPosition().getValueAsDouble()).getDegrees());
    Logger.recordOutput("GripperPivot/Erro", Rotation2d.fromRotations(m_motor.getPosition().getValueAsDouble()).minus(getCurrentAngle()).getDegrees());  
    Logger.recordOutput("GripperPivot/MotorVelocityRPS", m_motor.getVelocity().getValueAsDouble());
  }
}
