// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.swerve.BaseSwerveDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.utils.FieldPoint;
import java.util.function.Supplier;

/**
 * Type in a sentence.
 */
public class DriverRelativeSetAngleAndAxisDrive extends Command {
  private Gamepad m_driver;
  private BaseSwerveDrive m_swerveDrive;
  double m_magnitude;
  Supplier<Rotation2d> m_rotationSupplier; // A supplier is needed in case a DriveDirection is used - we probably won't know our alliance on start up

  /**
 * Type in a sentence.
 */
  public DriverRelativeSetAngleAndAxisDrive(Gamepad driver, BaseSwerveDrive swerveDrive, Supplier<Rotation2d> rotationSupplier, double magnitude) {
    m_driver = driver;
    m_swerveDrive = swerveDrive;
    m_magnitude = magnitude;
    m_rotationSupplier = rotationSupplier;

    addRequirements(m_swerveDrive);
  }

  /**
 * Type in a sentence.
 */
  public DriverRelativeSetAngleAndAxisDrive(Gamepad driver, BaseSwerveDrive swerveDrive, Rotation2d angle, double magnitude) {
    this(driver, swerveDrive, () -> angle, magnitude);
  }

  /**
 * Type in a sentence.
 */
  public DriverRelativeSetAngleAndAxisDrive(Gamepad driver, BaseSwerveDrive swerveDrive, FieldPoint fieldPoint, double magnitude) {
    this(driver, swerveDrive, () -> fieldPoint.getCurrentAlliancePose().relativeTo(swerveDrive.getPose()).getTranslation().getAngle(), magnitude);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_swerveDrive.driverModeInit();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveDrive.moveFieldRelativeAngle(MetersPerSecond.of(0), 
                                         SwerveConstants.MaxFreeSpeed.times(-m_driver.getSlewLeftX() * 0.3),
                                         m_rotationSupplier.get(), m_magnitude);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
