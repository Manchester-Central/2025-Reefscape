// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class SwerveModule2025 extends TalonFxAndCancoderSwerveModule {
    public SwerveModule2025(String nameString, Translation2d wheelPosition,
     SpeedControllerConfig speedControl, AngleControllerConfig angleControl,
     AbsoluteEncoderConfig absoEncoder, DriveConfig drivConfg) {
        super(nameString, wheelPosition, speedControl, angleControl, absoEncoder, drivConfg);
    }
    //
}
