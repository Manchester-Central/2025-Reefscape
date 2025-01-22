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
    DEPLOY;
  };

  /** Creates a new Intake. */
  public Intake() {
    super(IntakeState.START);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  protected void runStateMachine() {
    System.out.println(m_currentState);
    switch (m_currentState) {
      case START:
        m_currentState = IntakeState.STOW;
        break;

      case STOW:
        break;

      case DEPLOY:
        break;

      default:
        break;
    }
  }
}
