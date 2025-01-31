// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.subsystems.shared.ISubsystemState;
import frc.robot.subsystems.shared.StateBasedSubsystem;

public class Intake extends StateBasedSubsystem<Intake.IntakeState> {
  public enum IntakeState implements ISubsystemState {
    START,
    STOW,
    DEPLOY,
    HANDOFF_PREP,
    HANDOFF;
  };

  /** Creates a new Intake. */
  public Intake() {
    super(IntakeState.START);
  }

  @Override
  protected void runStateMachine() {
    // System.out.println(m_currentState);
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

  private void stowState() {}

  private void deployState() {}

  private void handoffPrepState() {}

  private void handoffState() {}
}
