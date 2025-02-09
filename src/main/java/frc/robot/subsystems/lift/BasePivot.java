// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import com.chaos131.pid.PIDFValue;
import com.chaos131.pid.PIDTuner;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.subsystems.lift.IdLift.IdLiftValues;
import frc.robot.utils.ChaosTalonFx;
import java.util.function.Supplier;

/** Add your docs here. */
public class BasePivot extends AbstractLiftPart {
  private double m_gearRatio = 10.0;
  private double m_jkgMetersSquared = 1.0;
  private Rotation2d m_targetAngle = Rotation2d.fromDegrees(120);
  private DCMotor m_dcMotor = DCMotor.getKrakenX60(1);
  private DCMotorSim m_motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(m_dcMotor, m_jkgMetersSquared, m_gearRatio),
          m_dcMotor,
          0.001,
          0.001);
  private ChaosTalonFx m_motor =
      new ChaosTalonFx(CanIdentifiers.BasePivotMotorCANID, m_gearRatio, m_motorSim, true);
  private CANcoder m_canCoder =
      new CANcoder(CanIdentifiers.BasePivotCANcoderCANID, CanIdentifiers.CTRECANBus);
  private PIDTuner m_pidTuner = new PIDTuner("BasePivot", true, 1.0, 0.001, 0.0, this::tunePids);

  /**
   * Creates a new BasePivot.
   *
   * @param idLiftValuesSupplier the supplier of lift values
   */
  public BasePivot(Supplier<IdLiftValues> idLiftValuesSupplier) {
    super(idLiftValuesSupplier);
    m_motor.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_motor.Configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_motor.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_motor.Configuration.CurrentLimits.SupplyCurrentLimit = 40;
    m_motor.Configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    m_motor.Configuration.Feedback.SensorToMechanismRatio = 10; // TODO: get real value
    m_motor.Configuration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
    m_motor.applyConfig();
  }

  private void tunePids(PIDFValue pidfValue) {
    m_motor.tunePid(pidfValue, 0.0);
  }

  /**
   * Sets the direct speed [-1.0, 1.0] of the motors.
   */
  public void setSpeed(double speed) {
    m_motor.set(speed);
  }

  /**
   * Sets the target angle and tries to drive there.
   */
  public void setTargetAngle(Rotation2d newAngle) {
    m_targetAngle = newAngle;
    m_motor.moveToPosition(newAngle.getDegrees());
  }

  public Rotation2d getCurrentAngle() {
    return Rotation2d.fromDegrees(
        m_motor.getPosition().getValueAsDouble()); // TODO get actual motor angle
  }

  /**
   * Checks if the angle is safe enough for other parts to move.
   */
  public boolean isSafeAngle() {
    return Math.abs(getCurrentAngle().minus(m_targetAngle).getDegrees()) < 10;
  }

  /**
   * Checks if the current angle is at the goal angle.
   */
  public boolean atTarget() {
    return Math.abs(getCurrentAngle().minus(m_targetAngle).getDegrees()) < 0.1;
  }

  @Override
  public void periodic() {
    m_pidTuner.tune();
  }

  @Override
  public void simulationPeriodic() {
    m_motor.simUpdate();
  }
}
