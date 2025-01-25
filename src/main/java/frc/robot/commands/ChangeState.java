// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.lift.IdLift.LiftState;
import java.util.Optional;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ChangeState extends Command {
  Optional<LiftState> m_idLiftState;
  Optional<IntakeState> m_intakeState;

  /** Creates a new uhhh. */
  public ChangeState() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public ChangeState setLift(LiftState newLiftState) {
    m_idLiftState = Optional.of(newLiftState);
    return this;
  }

  public ChangeState setIntake(IntakeState newIntakeState) {
    m_intakeState = Optional.of(newIntakeState);
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_idLiftState.isPresent()) {
      RobotContainer.m_idLift.changeState(m_idLiftState.get());
    }

    if (m_intakeState.isPresent()) {
      RobotContainer.m_intake.changeState(m_intakeState.get());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
