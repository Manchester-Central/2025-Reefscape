// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.MidLiftConstants.BasePivotConstants;
import frc.robot.Constants.MidLiftConstants.ExtenderConstants;
import frc.robot.Constants.MidLiftConstants.GripperPivotConstants;
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
  }

  public IdLiftValues getLiftValues() {
    IdLiftValues values = new IdLiftValues();
    values.basePivotAngle = m_basePivot.getCurrentAngle();
    values.gripperPivotAngle = m_gripperPivot.getCurrentAngle();
    values.gripperSpeed = m_gripper.getCurrentSpeed();
    values.extenderLength = m_extender.getCurrentLength();
    values.isBasePivotAtSafeAngle = m_basePivot.isSafeAngle();
    values.isExtenderAtSafeLength = m_extender.isSafeLength();
    return values;
  }

  private BasePivot m_basePivot = new BasePivot(this::getLiftValues);
  private Extender m_extender = new Extender(this::getLiftValues);
  private Gripper m_gripper = new Gripper(this::getLiftValues);
  private GripperPivot m_gripperPivot = new GripperPivot(this::getLiftValues);

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
  public IdLift() {
    super(LiftState.START);
  }

  @Override
  protected void runStateMachine() {
    // System.out.println(m_currentState);
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
    stowState(); // TODO: THINK ABOUT HP PICKUP
  }

  private void scoreL1State() {
    m_basePivot.setTargetAngle(BasePivotConstants.ScoreL1Angle);
    m_extender.setTargetLength(ExtenderConstants.ScoreL1LengthMeter);
    m_gripperPivot.setTargetAngle(GripperPivotConstants.ScoreL1Angle);
    m_gripper.setTargetSpeed(0.5);
  }

  private void scoreL2State() {
    m_basePivot.setTargetAngle(BasePivotConstants.ScoreL2Angle);
    m_extender.setTargetLength(ExtenderConstants.ScoreL2LengthMeter);
    m_gripperPivot.setTargetAngle(GripperPivotConstants.ScoreL2Angle);
    m_gripper.setTargetSpeed(0.5);
  }

  private void scoreL3State() {
    m_basePivot.setTargetAngle(BasePivotConstants.ScoreL3Angle);
    m_extender.setTargetLength(ExtenderConstants.ScoreL3LengthMeter);
    m_gripperPivot.setTargetAngle(GripperPivotConstants.ScoreL3Angle);
    m_gripper.setTargetSpeed(0.5);
  }

  private void scoreL4State() {
    m_basePivot.setTargetAngle(BasePivotConstants.ScoreL4Angle);
    m_extender.setTargetLength(ExtenderConstants.ScoreL4LengthMeter);
    m_gripperPivot.setTargetAngle(GripperPivotConstants.ScoreL4Angle);
    m_gripper.setTargetSpeed(-0.5);
  }
}
// RIP m_oldLift & m_oldGripper 2025-2025
