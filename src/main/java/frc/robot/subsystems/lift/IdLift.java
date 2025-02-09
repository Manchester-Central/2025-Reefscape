// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import com.chaos131.gamepads.Gamepad;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.MidLiftConstants.BasePivotConstants;
import frc.robot.Constants.MidLiftConstants.ExtenderConstants;
import frc.robot.Constants.MidLiftConstants.GripperPivotConstants;
import frc.robot.Robot;
import frc.robot.subsystems.shared.ISubsystemState;
import frc.robot.subsystems.shared.StateBasedSubsystem;

/** Add your docs here. */
public class IdLift extends StateBasedSubsystem<IdLift.LiftState> {
  public class IdLiftValues {
    public Rotation2d basePivotAngle;
    public Rotation2d gripperPivotAngle;
    public double gripperSpeed;
    public double extenderLength;
    public boolean isBasePivotAtSafeAngle;
    public boolean isExtenderAtSafeLength;
    public boolean hasCoralBackGripped;
    public boolean hasCoralFrontGripped;
  }

  public IdLiftValues getLiftValues() {
    IdLiftValues values = new IdLiftValues();
    values.basePivotAngle = m_basePivot.getCurrentAngle();
    values.gripperPivotAngle = m_gripperPivot.getCurrentAngle();
    values.gripperSpeed = m_gripper.getCurrentSpeed();
    values.extenderLength = m_extender.getCurrentLength();
    values.isBasePivotAtSafeAngle = m_basePivot.isSafeAngle();
    values.isExtenderAtSafeLength = m_extender.isSafeLength();
    values.hasCoralBackGripped = m_gripper.hasCoralBack();
    values.hasCoralFrontGripped = m_gripper.hasCoralFront();
    return values;
  }

  private BasePivot m_basePivot = new BasePivot(this::getLiftValues);
  private Extender m_extender = new Extender(this::getLiftValues);
  private Gripper m_gripper = new Gripper(this::getLiftValues);
  private GripperPivot m_gripperPivot = new GripperPivot(this::getLiftValues);
  private Gamepad m_operator;

  public enum LiftState implements ISubsystemState {
    MANUAL,
    START,
    STOW,
    INTAKE_FROM_FLOOR,
    INTAKE_FROM_HP, // Probably won't implement -Josh // nevermind -Josh
    SCORE_L1,
    SCORE_L2,
    SCORE_L3,
    SCORE_L4; // Might need many prep states
  }

  /** Creates a new Lift. */
  public IdLift(Gamepad operator) {
    super(LiftState.START);
    m_operator = operator;
  }

  @Override
  protected void runStateMachine() {
    // System.out.println(m_currentState);
    switch (m_currentState) {
      case MANUAL:
        manualState();
        break;
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
    if (Robot.isSimulation()) {
      m_currentState = LiftState.STOW;
    } else {
      m_currentState = LiftState.MANUAL;
    }
  }

  private void manualState() {
    m_basePivot.setSpeed(0.0);
    m_gripper.setTargetSpeed(0.0);
    m_gripperPivot.setSpeed(0.0);
    m_extender.setSpeed(m_operator.getRightY() * 0.3);
  }

  private void stowState() {
    m_basePivot.setTargetAngle(BasePivotConstants.StowAngle);
    m_extender.setTargetLength(ExtenderConstants.StowLengthMeter);
    m_gripperPivot.setTargetAngle(GripperPivotConstants.StowAngle);
    m_gripper.setTargetSpeed(0.0);
  }

  private void intakeFromFloorState() {
    m_basePivot.setTargetAngle(BasePivotConstants.HandoffAngle);
    m_extender.setTargetLength(ExtenderConstants.HandoffLengthMeter);
    m_gripperPivot.setTargetAngle(GripperPivotConstants.HandoffAngle);
    m_gripper.setTargetSpeed(0.5);
  }

  private void intakeFromHPState() {
    if (m_gripper.hasCoralBack() || m_gripper.hasCoralFront()) {
      m_currentState = LiftState.STOW;
      return;
    }
    m_basePivot.setTargetAngle(BasePivotConstants.hpIntakeAngle);
    m_extender.setTargetLength(ExtenderConstants.hpIntakeLengthMeter);
    m_gripperPivot.setTargetAngle(GripperPivotConstants.hpIntakeAngle);
    m_gripper.setTargetSpeed(0.5);
  }

  private void scoreL1State() {
    if (!(m_gripper.hasCoralBack() || m_gripper.hasCoralFront())) {
      m_currentState = LiftState.STOW;
      return;
    }
    m_basePivot.setTargetAngle(BasePivotConstants.ScoreL1Angle);
    m_extender.setTargetLength(ExtenderConstants.ScoreL1LengthMeter);
    m_gripperPivot.setTargetAngle(GripperPivotConstants.ScoreL1Angle);
    m_gripper.setTargetSpeed(0.5);
  }

  private void scoreL2State() {
    if (!(m_gripper.hasCoralBack() || m_gripper.hasCoralFront())) {
      m_currentState = LiftState.STOW;
      return;
    }
    m_basePivot.setTargetAngle(BasePivotConstants.ScoreL2Angle);
    m_extender.setTargetLength(ExtenderConstants.ScoreL2LengthMeter);
    m_gripperPivot.setTargetAngle(GripperPivotConstants.ScoreL2Angle);
    m_gripper.setTargetSpeed(0.5);
  }

  private void scoreL3State() {
    if (!(m_gripper.hasCoralBack() || m_gripper.hasCoralFront())) {
      m_currentState = LiftState.STOW;
      return;
    }
    m_basePivot.setTargetAngle(BasePivotConstants.ScoreL3Angle);
    m_extender.setTargetLength(ExtenderConstants.ScoreL3LengthMeter);
    m_gripperPivot.setTargetAngle(GripperPivotConstants.ScoreL3Angle);
    m_gripper.setTargetSpeed(0.5);
  }

  private void scoreL4State() {
    if (!(m_gripper.hasCoralBack() || m_gripper.hasCoralFront())) {
      m_currentState = LiftState.STOW;
      return;
    }
    m_basePivot.setTargetAngle(BasePivotConstants.ScoreL4Angle);
    m_extender.setTargetLength(ExtenderConstants.ScoreL4LengthMeter);
    m_gripperPivot.setTargetAngle(GripperPivotConstants.ScoreL4Angle);
    m_gripper.setTargetSpeed(-0.5);
  }
}
// RIP m_oldLift & m_oldGripper 2025-2025
