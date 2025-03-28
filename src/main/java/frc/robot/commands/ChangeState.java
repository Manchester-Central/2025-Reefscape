// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.Arm.ArmState;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * A command for changing ANY state on the robot.
 */
public class ChangeState extends Command {
  Optional<Supplier<ArmState>> m_armStateSupplier = Optional.empty();
  Optional<ArmState> m_armInterruptState = Optional.empty();

  /** Creates a new uhhh. */
  public ChangeState() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /**
   * Sets the state of the arm.
   */
  public ChangeState setArm(ArmState newArmState) {
    m_armStateSupplier = Optional.of(() -> newArmState);
    return this;
  }

  /**
   * Sets the state of the arm.
   */
  public ChangeState setArm(Supplier<ArmState> newArmStateSupplier) {
    m_armStateSupplier = Optional.of(newArmStateSupplier);
    return this;
  }

  /**
   * Builder function to handle a return to state when the command is interrupted.
   *
   * @param newArmState state to become
   * @return self
   */
  public ChangeState withArmInterrupt(ArmState newArmState) {
    m_armInterruptState = Optional.of(newArmState);
    return this;
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_armStateSupplier.isPresent()) {
      RobotContainer.m_arm.changeState(m_armStateSupplier.get().get());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted && m_armStateSupplier.isPresent() && m_armInterruptState.isPresent() 
        && RobotContainer.m_arm.getCurrentState() == m_armStateSupplier.get().get()) {

      RobotContainer.m_arm.changeState(m_armInterruptState.get());
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DriverStation.isAutonomous();
  }
}
