// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shared;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public abstract class StateBasedSubsystem<T extends SubsystemState> extends SubsystemBase {
  private T m_currentState;
  protected Timer m_stateTimer = new Timer();

  /**
   * The base contrusctor for all state based subsystems.
   *
   * @param startState the state to start the state machine in.
   */
  protected StateBasedSubsystem(T startState) {
    m_currentState = startState;
    m_stateTimer.start();
    this.setDefaultCommand(new RunCommand(() -> runStateMachine(), this));
  }

  /**
   * The function that must be called every loop to run the state machine.
   */
  protected abstract void runStateMachine();

  /**
   * Changes the current state of the state machine.
   *
   * @param newState the new state
   */
  public void changeState(T newState) {
    if (newState != m_currentState) {
      m_stateTimer.restart();
    }

    m_currentState = newState;
  }

  public T getCurrentState() {
    return m_currentState;
  }
}
