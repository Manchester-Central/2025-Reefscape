// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.arm.Arm.ArmState;
import java.util.Optional;

/**
 * A command for awaiting ANY state on the robot.
 */
public class WaitForState extends Command {
  Optional<ArmState> m_armState = Optional.empty();
  Optional<IntakeState> m_intakeState = Optional.empty();

  /** Creates a new Wait Command. */
  public WaitForState() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /**
   *  the state of the arm to wait for.
   */
  public WaitForState forArmState(ArmState newArmState) {
    m_armState = Optional.of(newArmState);
    return this;
  }

  /**
   *  the state of the intake to wait for.
   */
  public WaitForState forIntakeState(IntakeState newIntakeState) {
    m_intakeState = Optional.of(newIntakeState);
    return this;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isArmStateGood = m_armState.isPresent() ? RobotContainer.m_arm.getCurrentState() == m_armState.get() : true;
    boolean isIntakeStateGood = m_intakeState.isPresent() ? RobotContainer.m_intake.getCurrentState() == m_intakeState.get() : true;
    return isArmStateGood && isIntakeStateGood;
  }
}
