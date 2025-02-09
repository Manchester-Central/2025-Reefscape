// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chaos131.pid.PIDFValue;
import com.chaos131.pid.PIDTuner;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.shared.StateBasedSubsystem;
import frc.robot.subsystems.shared.SubsystemState;
import frc.robot.utils.ChaosTalonFx;

/**
 * The Intake subsystem for grabbing coral off the ground.
 */
public class Intake extends StateBasedSubsystem<Intake.IntakeState> {

  /**
   * The possible states of the Intake's state machine.
   */
  public enum IntakeState implements SubsystemState {
    START,
    STOW,
    DEPLOY,
    HANDOFF_PREP,
    HANDOFF;
  }

  private double m_gearRatio = 10.0;
  private double m_jkgMetersSquared = 1.0;
  private Rotation2d m_targetAngle = Rotation2d.fromDegrees(120);
  private double m_targetSpeed = 0.0;
  private DCMotor m_pivotDcMotor = DCMotor.getKrakenX60(2);
  private DCMotorSim m_pivotMotorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(m_pivotDcMotor, m_jkgMetersSquared, m_gearRatio),
          m_pivotDcMotor,
          0.001,
          0.001);
  private ChaosTalonFx m_pivotMotor1 = new ChaosTalonFx(10, m_gearRatio, m_pivotMotorSim, true);
  private ChaosTalonFx m_pivotMotor2 = new ChaosTalonFx(11, m_gearRatio, m_pivotMotorSim, false);
  private PIDTuner m_pidTuner = new PIDTuner("IntakePivot", true, 0.1, 0.001, 0.0, this::tunePids);

  /** Creates a new Intake. */
  public Intake() {
    super(IntakeState.START);
    m_pivotMotor1.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_pivotMotor1.Configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_pivotMotor1.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_pivotMotor1.Configuration.CurrentLimits.SupplyCurrentLimit = 40;
    m_pivotMotor1.Configuration.Feedback.FeedbackSensorSource =
        FeedbackSensorSourceValue.RotorSensor;
    m_pivotMotor1.Configuration.Feedback.SensorToMechanismRatio = 0.5; // TODO: get real value
    m_pivotMotor1.Configuration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
    m_pivotMotor1.applyConfig();

    m_pivotMotor2.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_pivotMotor2.Configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_pivotMotor2.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_pivotMotor2.Configuration.CurrentLimits.SupplyCurrentLimit = 40;
    m_pivotMotor2.Configuration.Feedback.FeedbackSensorSource =
        FeedbackSensorSourceValue.RotorSensor;
    m_pivotMotor2.Configuration.Feedback.SensorToMechanismRatio = 0.5; // TODO: get real value
    m_pivotMotor2.Configuration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
    m_pivotMotor2.applyConfig();
  }

  @Override
  protected void runStateMachine() {
    switch (m_currentState) {
      case START:
        m_currentState = IntakeState.STOW;
        break;

      case STOW:
        stowState();
        break;

      case DEPLOY:
        deployState();
        break;

      case HANDOFF_PREP:
        handoffPrepState();
        break;

      case HANDOFF:
        handoffState();
        break;

      default:
        break;
    }
  }

  private void stowState() {
    setTargetAngle(IntakeConstants.StowAngle);
    setTargetSpeed(IntakeConstants.StowSpeed);
  }

  private void deployState() {
    setTargetAngle(IntakeConstants.DeployAngle);
    setTargetSpeed(IntakeConstants.DeploySpeed);
  }

  private void handoffPrepState() {
    setTargetAngle(IntakeConstants.HandoffAngle);
    setTargetSpeed(IntakeConstants.HandoffPrepSpeed);
  }

  private void handoffState() {
    setTargetAngle(IntakeConstants.HandoffAngle);
    setTargetSpeed(IntakeConstants.HandoffSpeed);
  }

  private void tunePids(PIDFValue pidfValue) {
    m_pivotMotor1.tunePid(pidfValue, 0.0);
    m_pivotMotor2.tunePid(pidfValue, 0.0);
  }

  public void setTargetSpeed(double newSpeed) {
    m_targetSpeed = newSpeed;
  }

  public double getCurrentSpeed() {
    return m_targetSpeed;
  }

  /**
   * Sets the target angle and tries to drive there.
   */
  public void setTargetAngle(Rotation2d newAngle) {
    m_targetAngle = newAngle;
    m_pivotMotor1.moveToPosition(newAngle.getDegrees());
    m_pivotMotor2.moveToPosition(newAngle.getDegrees());
  }

  public Rotation2d getCurrentAngle() {
    return Rotation2d.fromDegrees(
        m_pivotMotor1.getPosition().getValueAsDouble()); // TODO get actual motor angle
  }

  @Override
  public void periodic() {
    m_pidTuner.tune();
  }

  @Override
  public void simulationPeriodic() {
    m_pivotMotor1.simUpdate();
    m_pivotMotor2.simUpdate();
  }
}
