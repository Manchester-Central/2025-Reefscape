// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class SwerveModule2025 extends TalonFxAndCancoderSwerveModule {
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
      Angle angleEncoderOffset) {
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
            canCoderCanId, SwerveConstants.InvertedEncoder, Rotation2d.fromDegrees(angleEncoderOffset.in(Degrees))),
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
    double updateFrequency = 250.0;
    m_speedController.getPosition().setUpdateFrequency(updateFrequency);
    m_speedController.getRotorPosition().setUpdateFrequency(updateFrequency);
    m_speedController.getVelocity().setUpdateFrequency(updateFrequency);
    m_speedController.getRotorVelocity().setUpdateFrequency(updateFrequency);
    m_speedController.getMotorVoltage().setUpdateFrequency(updateFrequency);
    m_speedController.getSupplyCurrent().setUpdateFrequency(updateFrequency);

    m_angleController.getRotorVelocity().setUpdateFrequency(updateFrequency);
    m_angleController.getMotorVoltage().setUpdateFrequency(updateFrequency);
    m_angleController.getSupplyCurrent().setUpdateFrequency(updateFrequency);
    m_angleController.getPosition().setUpdateFrequency(updateFrequency);
  }

  /**
   * To change the ramp rate period on the fly.
   *
   * @param newRate time in seconds
   */
  public void setRampRatePeriod(double newRate) {
    m_speedConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = newRate;
    m_speedConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = newRate;
    m_speedController.getConfigurator().apply(m_speedConfig.ClosedLoopRamps);
  }

  /**
   * To change between brake and coast mode.
   * 
   * @param mode new mode
   */
  public void setSpeedMotorNeutralMode(NeutralModeValue mode) {
    m_speedConfig.MotorOutput.NeutralMode = mode;
    m_speedController.getConfigurator().apply(m_speedConfig.MotorOutput);
  }

  public void setSupplyCurrentLimit(int currentLimit) {
    m_speedConfig.CurrentLimits.SupplyCurrentLimit = currentLimit;
    m_speedController.getConfigurator().apply(m_speedConfig.CurrentLimits);
  }
}
