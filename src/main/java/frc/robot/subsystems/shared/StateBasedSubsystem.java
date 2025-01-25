// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shared;

import com.chaos131.gamepads.Gamepad;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;

/** Add your docs here. */
public abstract class StateBasedSubsystem<TState extends ISubsystemState> extends SubsystemBase {
  public static Gamepad DriverController;
  public static Gamepad OperatorController;
  public static Intake IntakeSystem;
  public static Gripper GripperSystem;
  public static Lift LiftSystem;
  protected TState m_currentState;

  protected StateBasedSubsystem(TState startState) {
    m_currentState = startState;
    this.setDefaultCommand(new RunCommand(() -> runStateMachine(), this));
  }

  protected abstract void runStateMachine();
}
