// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.lift.IdLift.LiftState;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * A command for changing ANY state on the robot.
 */
public class ChangeState extends Command {
  Optional<Supplier<LiftState>> m_idLiftStateSupplier = Optional.empty();
  Optional<IntakeState> m_intakeState = Optional.empty();

  /** Creates a new uhhh. */
  public ChangeState() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /**
   * Sets the state of the lift.
   */
  public ChangeState setLift(LiftState newLiftState) {
    m_idLiftStateSupplier = Optional.of(() -> newLiftState);
    return this;
  }

    /**
   * Sets the state of the lift.
   */
  public ChangeState setLift(Supplier<LiftState> newLiftStateSupplier) {
    m_idLiftStateSupplier = Optional.of(newLiftStateSupplier);
    return this;
  }

  /**
   * Sets the state of the intake.
   */
  public ChangeState setIntake(IntakeState newIntakeState) {
    m_intakeState = Optional.of(newIntakeState);
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_idLiftStateSupplier.isPresent()) {
      RobotContainer.m_idLift.changeState(m_idLiftStateSupplier.get().get());
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
    return true;
  }
}
