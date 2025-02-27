// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Constants.SwerveConstants;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/** Add your docs here. */
public class SwerveModule2025 extends TalonFxAndCancoderSwerveModule {
  private SwerveModuleSimulation m_simulation;

  /**
   * Creates a swerve module for the 2025 bot.
   */
  public SwerveModule2025(
      String nameString,
      Translation2d wheelPosition,
      int speedCanId,
      int angleCanId,
      int canCoderCanId,
      InvertedValue speedDirection,
      Rotation2d angleEncoderOffset) {
    super(
        nameString,
        CanIdentifiers.CTRECANBus,
        wheelPosition,
        new SpeedControllerConfig(
            speedCanId,
            speedDirection,
            SwerveConstants.SpeedGearRatio,
            SwerveConstants.SpeedCircumference),
        new AngleControllerConfig(
            angleCanId, SwerveConstants.InvertedAngle, SwerveConstants.AngleGearRatio),
        new AbsoluteEncoderConfig(
            canCoderCanId, SwerveConstants.InvertedEncoder, angleEncoderOffset),
        new DriveConfig(
            SwerveConstants.DriverRampRatePeriod, SwerveConstants.AutonomousRampRatePeriod));

    m_speedConfig.CurrentLimits = new CurrentLimitsConfigs();
    m_speedConfig.CurrentLimits.SupplyCurrentLimit = 35;
    m_speedConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_speedConfig.CurrentLimits.SupplyCurrentLowerLimit = 85;
    m_speedConfig.CurrentLimits.SupplyCurrentLowerTime = 0.01;
    m_speedConfig.CurrentLimits.StatorCurrentLimit = 75;
    m_speedConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    m_speedController.getConfigurator().apply(m_speedConfig.CurrentLimits);
  }

  /**
   * Inits the sim in the module (after the swerve drive has been instantiated).
   */
  public void initSim(SwerveModuleSimulation simDrive) {
    m_simulation = simDrive;
  }
}
