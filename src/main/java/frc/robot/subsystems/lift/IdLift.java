// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import frc.robot.subsystems.shared.ISubsystemState;
import frc.robot.subsystems.shared.StateBasedSubsystem;

/** Add your docs here. */
public class IdLift extends StateBasedSubsystem<IdLift.LiftState> {
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
  public IdLift() {
    super(LiftState.START);
  }

  @Override
  protected void runStateMachine() {
    System.out.println(m_currentState);
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

  private void startState() {}

  private void stowState() {}

  private void intakeFromFloorState() {}

  private void intakeFromHPState() {}

  private void scoreL1State() {}

  private void scoreL2State() {}

  private void scoreL3State() {}

  private void scoreL4State() {}
}
// RIP m_oldLift & m_oldGripper 2025-2025
