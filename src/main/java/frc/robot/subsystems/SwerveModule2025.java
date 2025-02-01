// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule;
import edu.wpi.first.math.geometry.Translation2d;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/** Add your docs here. */
public class SwerveModule2025 extends TalonFxAndCancoderSwerveModule {
  public SwerveModuleSimulation m_simulation;

  public SwerveModule2025(
      String nameString,
      Translation2d wheelPosition,
      SpeedControllerConfig speedControl,
      AngleControllerConfig angleControl,
      AbsoluteEncoderConfig absoEncoder,
      DriveConfig drivConfg,
      SwerveModuleSimulation simulation) {
    super(nameString, wheelPosition, speedControl, angleControl, absoEncoder, drivConfg);
    m_simulation = simulation;
  }

  public SwerveModule2025(
      String nameString,
      Translation2d wheelPosition,
      SpeedControllerConfig speedControl,
      AngleControllerConfig angleControl,
      AbsoluteEncoderConfig absoEncoder,
      DriveConfig drivConfg) {
    super(nameString, wheelPosition, speedControl, angleControl, absoEncoder, drivConfg);
  }

  public void initSim(SwerveModuleSimulation simDrive) {
    m_simulation = simDrive;
  }
}
