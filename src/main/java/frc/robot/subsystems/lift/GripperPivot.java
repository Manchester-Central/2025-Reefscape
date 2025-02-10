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
import frc.robot.Constants.MidLiftConstants.GripperPivotConstants;
import frc.robot.Constants.MidLiftConstants.LiftPoses;
import frc.robot.subsystems.lift.IdLift.IdLiftValues;
import frc.robot.utils.ChaosTalonFx;
import java.util.function.Supplier;

/** Add your docs here. */
public class GripperPivot extends AbstractLiftPart {
  private double m_gearRatio = 40.0;
  private double m_jkgMetersSquared = 0.1;
  private Rotation2d m_targetAngle = Rotation2d.fromDegrees(120);
  private DCMotor m_dcMotor = DCMotor.getKrakenX60(1);
  private DCMotorSim m_motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(m_dcMotor, m_jkgMetersSquared, m_gearRatio),
          m_dcMotor,
          0.001,
          0.001);
  private ChaosTalonFx m_motor =
      new ChaosTalonFx(CanIdentifiers.GripperPivotMotorCANID, m_gearRatio, m_motorSim, true);
  private CANcoder m_canCoder =
      new CANcoder(CanIdentifiers.GripperPivotCANCoderCANID, CanIdentifiers.CTRECANBus);
  private PIDTuner m_pidTuner = new PIDTuner("GripperPivot", true, 1.0, 0.001, 0.0, this::tunePid);

  /**
   * Creates a new GripperPivot.
   *
   * @param idLiftValuesSupplier the supplier of lift values
   */
  public GripperPivot(Supplier<IdLiftValues> idLiftValuesSupplier) {
    super(idLiftValuesSupplier);
    m_motor.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_motor.Configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_motor.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_motor.Configuration.CurrentLimits.SupplyCurrentLimit = 40;
    m_motor.Configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    m_motor.Configuration.Feedback.SensorToMechanismRatio = 1; // TODO: get real value
    m_motor.Configuration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
    m_motor.applyConfig();
  }

  private void tunePid(PIDFValue pidfValue) {
    m_motor.tunePid(pidfValue, 0.0);
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

    if (!getLiftValues().isBasePivotAtSafeAngle || !getLiftValues().isExtenderAtSafeLength) {
      newAngle = LiftPoses.Stow.getGripperPivotAngle();
    }
    m_targetAngle = newAngle;
    m_motor.moveToPosition(newAngle.getDegrees());
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

  public Rotation2d getCurrentAngle() {
    return Rotation2d.fromDegrees(
        m_motor.getPosition().getValueAsDouble()); // TODO get actual motor angle
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
