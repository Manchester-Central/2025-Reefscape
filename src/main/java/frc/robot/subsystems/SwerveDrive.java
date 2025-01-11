// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import org.ejml.equation.Variable;

import com.chaos131.swerve.BaseSwerveDrive;
import com.chaos131.swerve.BaseSwerveModule;
import com.chaos131.swerve.SwerveConfigs;
import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule.AbsoluteEncoderConfig;
import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule.AngleControllerConfig;
import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule.DriveConfig;
import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule.SpeedControllerConfig;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class SwerveDrive extends BaseSwerveDrive {

    private SwerveDrive(BaseSwerveModule[] swerveModules, SwerveConfigs swerveConfigs,
            Supplier<Rotation2d> getRotation) {
        super(swerveModules, swerveConfigs, getRotation);
    }

    public static SwerveDrive SeparateConstructor(Pigeon2 gyrPigeon2) {
        SwerveConfigs swerveConfigs = new SwerveConfigs();
        SwerveModule2025 frontLeftSwerveModule = new SwerveModule2025("FL",
                new Translation2d(SwerveConstants.FLModoffsetX, SwerveConstants.FLModoffsetY),
                new SpeedControllerConfig(SwerveConstants.FLSpeedCANID, SwerveConstants.FLInvertedSpeed,
                        SwerveConstants.FLSpeedGearRatio, SwerveConstants.FLSpeedCircumference),
                new AngleControllerConfig(SwerveConstants.FLAngleCANID, SwerveConstants.FLInvertedAngle,
                        SwerveConstants.FLAngleGearRatio),
                new AbsoluteEncoderConfig(SwerveConstants.FLAbsoEncoCANID, SwerveConstants.FLInvertedEncoder,
                        SwerveConstants.FLAngleEncoderOffset),
                new DriveConfig(SwerveConstants.FLDriverRampRatePeriod, SwerveConstants.FLAutonomousRampRatePeriod));
        SwerveModule2025 frontRightSwerveModule = new SwerveModule2025("FR",
                new Translation2d(SwerveConstants.FRModoffsetX, SwerveConstants.FRModoffsetY),
                new SpeedControllerConfig(SwerveConstants.FRSpeedCANID, SwerveConstants.FRInvertedSpeed,
                        SwerveConstants.FRSpeedGearRatio, SwerveConstants.FRSpeedCircumference),
                new AngleControllerConfig(SwerveConstants.FRAngleCANID, SwerveConstants.FRInvertedAngle,
                        SwerveConstants.FRAngleGearRatio),
                new AbsoluteEncoderConfig(SwerveConstants.FRAbsoEncoCANID, SwerveConstants.FRInvertedEncoder,
                        SwerveConstants.FRAngleEncoderOffset),
                new DriveConfig(SwerveConstants.FRDriverRampRatePeriod, SwerveConstants.FRAutonomousRampRatePeriod));
        SwerveModule2025 backLeftSwerveModule = new SwerveModule2025("BL",
                new Translation2d(SwerveConstants.BLModoffsetX, SwerveConstants.BLModoffsetY),
                new SpeedControllerConfig(SwerveConstants.BLSpeedCANID, SwerveConstants.BLInvertedSpeed,
                        SwerveConstants.BLSpeedGearRatio, SwerveConstants.BLSpeedCircumference),
                new AngleControllerConfig(SwerveConstants.BLAngleCANID, SwerveConstants.BLInvertedAngle,
                        SwerveConstants.BLAngleGearRatio),
                new AbsoluteEncoderConfig(SwerveConstants.BLAbsoEncoCANID, SwerveConstants.BLInvertedEncoder,
                        SwerveConstants.BLAngleEncoderOffset),
                new DriveConfig(SwerveConstants.BLDriverRampRatePeriod, SwerveConstants.BLAutonomousRampRatePeriod));
        SwerveModule2025 backRightSwerveModule = new SwerveModule2025("BR",
                new Translation2d(SwerveConstants.BRModoffsetX, SwerveConstants.BRModoffsetY),
                new SpeedControllerConfig(SwerveConstants.BRSpeedCANID, SwerveConstants.BRInvertedSpeed,
                        SwerveConstants.BRSpeedGearRatio, SwerveConstants.BRSpeedCircumference),
                new AngleControllerConfig(SwerveConstants.BRAngleCANID, SwerveConstants.BRInvertedAngle,
                        SwerveConstants.BRAngleGearRatio),
                new AbsoluteEncoderConfig(SwerveConstants.BRAbsoEncoCANID, SwerveConstants.BRInvertedEncoder,
                        SwerveConstants.BRAngleEncoderOffset),
                new DriveConfig(SwerveConstants.BRDriverRampRatePeriod, SwerveConstants.BRAutonomousRampRatePeriod));
        SwerveModule2025[] swerveModule2025s = { frontLeftSwerveModule, frontRightSwerveModule, backLeftSwerveModule,
                backRightSwerveModule };
        SwerveDrive swerveDrive = new SwerveDrive(swerveModule2025s, swerveConfigs, () -> gyrPigeon2.getRotation2d());
        return swerveDrive;
    }

}
