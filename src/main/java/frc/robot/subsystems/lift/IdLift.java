// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import com.chaos131.robot.ChaosRobot.Mode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.shared.ISubsystemState;
import frc.robot.subsystems.shared.StateBasedSubsystem;

/** Add your docs here. */
public class IdLift extends StateBasedSubsystem<IdLift.LiftState> {
  public class IdLiftValues {
    public Rotation2d basePivotAngle;
    public Rotation2d gripperPivotAngle;
    public double gripperSpeed;
    public double extenderLength;
  }

  public IdLiftValues getLiftValues() {
    IdLiftValues values = new IdLiftValues();
    values.basePivotAngle = m_basePivot.getCurrentAngle();
    values.gripperPivotAngle = m_gripperPivot.getCurrentAngle();
    values.gripperSpeed = m_gripper.getCurrentSpeed();
    values.extenderLength = m_extender.getCurrentLength();
    return values;
  }

  public TalonFX m_talonfx;
  public CANcoder m_CANcoder;
  public DCMotor m_DCMotor;
  public ElevatorSim m_ElevatorSim;
  public final double GearRatio = 1.54;
  public final double ElevatorMassKg = 15.0;

  public TalonFXSimState m_simMotor;
  public Mode m_mode;

  private BasePivot m_basePivot = new BasePivot();
  private Extender m_extender = new Extender();
  private Gripper m_gripper = new Gripper();
  private GripperPivot m_gripperPivot = new GripperPivot();

  public enum LiftState implements ISubsystemState {
    START,
    STOW,
    INTAKE_FROM_FLOOR,
    INTAKE_FROM_HP, // Probably won't implement -Josh
    SCORE_L1,
    SCORE_L2,
    SCORE_L3,
    SCORE_L4; // Might need many prep states
  }

  /** Creates a new Lift. */
  public IdLift(Mode mode) {
    super(LiftState.START);
    m_mode = mode;
    m_talonfx = new TalonFX(0);
    m_CANcoder = new CANcoder(0);
    m_DCMotor = DCMotor.getKrakenX60(1);

    m_simMotor = m_talonfx.getSimState();
    m_ElevatorSim = new ElevatorSim(m_DCMotor, GearRatio, ElevatorMassKg, 0.076, 0, 2, false, 0);
  }

  private void SimUpdate() {
    m_ElevatorSim.update(0.02);
  }

  @Override
  protected void runStateMachine() {
    System.out.println(m_currentState);
    if (m_mode == Mode.SIM) {
      this.SimUpdate();
    }
    switch (m_currentState) {
      case START:
        startState();
        break;
      case STOW:
        stowState();
        break;
      case INTAKE_FROM_FLOOR:
        intakeFromFloorState();
        break;
      case INTAKE_FROM_HP:
        intakeFromHPState();
        break;
      case SCORE_L1:
        scoreL1State();
        break;
      case SCORE_L2:
        scoreL2State();
        break;
      case SCORE_L3:
        scoreL3State();
        break;
      case SCORE_L4:
        scoreL4State();
        break;
    }
  }

  private void startState() {
    m_currentState = LiftState.STOW;
  }

  private void stowState() {
    if (m_mode == Mode.SIM) {
      m_extender.setTargetLength(m_ElevatorSim.getPositionMeters());
      m_ElevatorSim.setState(
          m_ElevatorSim.getPositionMeters(), (0.7 - m_ElevatorSim.getPositionMeters()) * 20);
    } else {
      m_extender.setTargetLength(0.7);
    }
    m_basePivot.setTargetAngle(Rotation2d.fromDegrees(120));
    m_gripperPivot.setTargetAngle(Rotation2d.fromDegrees(90));
    m_gripper.setTargetSpeed(0.0);
  }

  private void intakeFromFloorState() {
    if (m_mode == Mode.SIM) {
      m_extender.setTargetLength(m_ElevatorSim.getPositionMeters());
      m_ElevatorSim.setState(
          m_ElevatorSim.getPositionMeters(), (0.4 - m_ElevatorSim.getPositionMeters()) * 20);
    } else {
      m_extender.setTargetLength(0.4);
    }
    m_basePivot.setTargetAngle(Rotation2d.fromDegrees(136));
    m_gripperPivot.setTargetAngle(Rotation2d.fromDegrees(51));
    m_gripper.setTargetSpeed(0.5);
  }

  private void intakeFromHPState() {
    stowState(); // TODO: THINK ABOUT HP PICKUP
  }

  private void scoreL1State() {
    if (m_mode == Mode.SIM) {
      m_extender.setTargetLength(m_ElevatorSim.getPositionMeters());
      m_ElevatorSim.setState(
          m_ElevatorSim.getPositionMeters(), (0.97 - m_ElevatorSim.getPositionMeters()) * 20);
    } else {
      m_extender.setTargetLength(0.97);
    }
    m_basePivot.setTargetAngle(Rotation2d.fromDegrees(126));
    m_gripperPivot.setTargetAngle(Rotation2d.fromDegrees(110));
    m_gripper.setTargetSpeed(0.5);
  }

  private void scoreL2State() {
    if (m_mode == Mode.SIM) {
      m_extender.setTargetLength(m_ElevatorSim.getPositionMeters());
      m_ElevatorSim.setState(
          m_ElevatorSim.getPositionMeters(), (1.0 - m_ElevatorSim.getPositionMeters()) * 20);
    } else {
      m_extender.setTargetLength(1.0);
    }
    m_basePivot.setTargetAngle(Rotation2d.fromDegrees(119));
    m_gripperPivot.setTargetAngle(Rotation2d.fromDegrees(117));
    m_gripper.setTargetSpeed(0.5);
  }

  private void scoreL3State() {
    if (m_mode == Mode.SIM) {
      m_extender.setTargetLength(m_ElevatorSim.getPositionMeters());
      m_ElevatorSim.setState(
          m_ElevatorSim.getPositionMeters(), (1.31 - m_ElevatorSim.getPositionMeters()) * 20);
    } else {
      m_extender.setTargetLength(1.31);
    }
    m_basePivot.setTargetAngle(Rotation2d.fromDegrees(110));
    m_gripperPivot.setTargetAngle(Rotation2d.fromDegrees(126));
    m_gripper.setTargetSpeed(0.5);
  }

  private void scoreL4State() {
    if (m_mode == Mode.SIM) {
      m_extender.setTargetLength(m_ElevatorSim.getPositionMeters());
      m_ElevatorSim.setState(
          m_ElevatorSim.getPositionMeters(), (1.8 - m_ElevatorSim.getPositionMeters()) * 20);
    } else {
      m_extender.setTargetLength(1.8);
    }
    m_basePivot.setTargetAngle(Rotation2d.fromDegrees(110));
    m_gripperPivot.setTargetAngle(Rotation2d.fromDegrees(-51));
    m_gripper.setTargetSpeed(-0.5);
  }
}
// RIP m_oldLift & m_oldGripper 2025-2025
