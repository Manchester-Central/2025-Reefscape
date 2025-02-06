// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Constants.SwerveConstants;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/** Add your docs here. */
public class SwerveModule2025 extends TalonFxAndCancoderSwerveModule {
  private SwerveModuleSimulation m_simulation;

  public SwerveModule2025(
      String nameString,
      Translation2d wheelPosition,
      int speedCANID,
      int angleCANID,
      int absoEncoCANID,
      InvertedValue invertedSpeed,
      Rotation2d angleEncoderOffset) {
    super(
        nameString,
        CanIdentifiers.CTRECANBus,
        wheelPosition,
        new SpeedControllerConfig(
            speedCANID,
            invertedSpeed,
            SwerveConstants.SpeedGearRatio,
            SwerveConstants.SpeedCircumference),
        new AngleControllerConfig(
            angleCANID, SwerveConstants.InvertedAngle, SwerveConstants.AngleGearRatio),
        new AbsoluteEncoderConfig(
            absoEncoCANID, SwerveConstants.InvertedEncoder, angleEncoderOffset),
        new DriveConfig(
            SwerveConstants.DriverRampRatePeriod, SwerveConstants.AutonomousRampRatePeriod));
  }

  public void initSim(SwerveModuleSimulation simDrive) {
    m_simulation = simDrive;
  }
}
