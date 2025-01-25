// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shared;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public abstract class StateBasedSubsystem<TState extends ISubsystemState> extends SubsystemBase {
  protected TState m_currentState;

  protected StateBasedSubsystem(TState startState) {
    m_currentState = startState;
    this.setDefaultCommand(new RunCommand(() -> runStateMachine(), this));
  }

  protected abstract void runStateMachine();

  public void changeState(TState newState) {
    m_currentState = newState;
  }
}
