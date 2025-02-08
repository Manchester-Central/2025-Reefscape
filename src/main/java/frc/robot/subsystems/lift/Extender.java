// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import com.chaos131.pid.PIDFValue;
import com.chaos131.pid.PIDTuner;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Constants.MidLiftConstants.ExtenderConstants;
import frc.robot.subsystems.lift.IdLift.IdLiftValues;
import frc.robot.utils.ChaosTalonFx;
import java.util.function.Supplier;

/** Add your docs here. */
public class Extender extends AbstractLiftPart {
  private double m_targetLength = 1;
  private double kGearRatio = 10.0;
  private double kJkgMetersSquared = 1.0;
  private DCMotor m_dcMotor = DCMotor.getKrakenX60(1);
  private DCMotorSim m_motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(m_dcMotor, kJkgMetersSquared, kGearRatio),
          m_dcMotor,
          0.001,
          0.001);
  private ChaosTalonFx m_motor1 =
      new ChaosTalonFx(CanIdentifiers.ExtenderMotorCANID, kGearRatio, m_motorSim, true);
  // private ChaosTalonFx m_motor2 = new ChaosTalonFx(5, kGearRatio, m_motorSim, false);
  private PIDTuner m_pidTuner = new PIDTuner("Extender", true, 1.0, 0.001, 0.0, this::tunePIDs);

  public Extender(Supplier<IdLiftValues> idLiftValuesSupplier) {
    super(idLiftValuesSupplier);

    m_motor1.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_motor1.Configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_motor1.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_motor1.Configuration.CurrentLimits.SupplyCurrentLimit = 40;
    m_motor1.Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    m_motor1.Configuration.CurrentLimits.StatorCurrentLimit = 40;
    m_motor1.Configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    m_motor1.Configuration.Feedback.SensorToMechanismRatio = 10; // TODO: get real value
    m_motor1.Configuration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
    m_motor1.applyConfig();

    // m_motor2.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // m_motor2.Configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // m_motor2.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    // m_motor2.Configuration.CurrentLimits.SupplyCurrentLimit = 40;
    // m_motor2.Configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    // m_motor2.Configuration.Feedback.SensorToMechanismRatio = 10; // TODO: get real value
    // m_motor2.Configuration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
    // m_motor2.applyConfig();
  }

  public void tunePIDs(PIDFValue pidfValue) {
    m_motor1.tunePID(pidfValue, 0.0);
    // m_motor2.tunePID(pidfValue, 0.0);
  }

  public void setTargetLength(double newLength) {
    if (!getLiftValues().isBasePivotAtSafeAngle) {
      newLength = ExtenderConstants.StowLengthMeter;
    }
    m_targetLength = newLength;
    m_motor1.moveToPosition(newLength);
    // m_motor2.moveToPosition(newLength);
  }

  public void setSpeed(double speed) {
    m_motor1.set(speed);
  }

  public double getCurrentLength() {
    return m_motor1.getPosition().getValueAsDouble();
  }

  public boolean isSafeLength() {
    return Math.abs(getCurrentLength() - m_targetLength) < 0.2;
  }

  @Override
  public void periodic() {
    m_pidTuner.tune();
  }

  @Override
  public void simulationPeriodic() {
    m_motor1.simUpdate();
    // m_motor2.simUpdate();
  }
}
