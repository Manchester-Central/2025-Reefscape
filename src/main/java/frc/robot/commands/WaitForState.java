// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.lift.IdLift.LiftState;
import java.util.Optional;

/**
 * A command for changing ANY state on the robot.
 */
public class WaitForState extends Command {
  Optional<LiftState> m_idLiftState = Optional.empty();
  Optional<IntakeState> m_intakeState = Optional.empty();

  /** Creates a new uhhh. */
  public WaitForState() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /**
   * Sets the state of the lift.
   */
  public WaitForState setLift(LiftState newLiftState) {
    m_idLiftState = Optional.of(newLiftState);
    return this;
  }

  /**
   * Sets the state of the intake.
   */
  public WaitForState setIntake(IntakeState newIntakeState) {
    m_intakeState = Optional.of(newIntakeState);
    return this;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isLiftStateGood = m_idLiftState.isPresent() ? RobotContainer.m_idLift.getCurrentState() == m_idLiftState.get() : true;
    boolean isIntakeStateGood = m_intakeState.isPresent() ? RobotContainer.m_intake.getCurrentState() == m_intakeState.get() : true;
    return isLiftStateGood && isIntakeStateGood;
  }
}
