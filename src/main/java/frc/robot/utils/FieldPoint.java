// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.chaos131.poses.MirroredDrivePose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class FieldPoint extends MirroredDrivePose {

    public static final FieldPoint processor=new FieldPoint("processor", new Pose2d(5.988,0,Rotation2d.fromDegrees(90)));

    public FieldPoint (String name, Pose2d pose ) {
        super(17.524,Alliance.Blue, name, pose);
        makeredpose();
    }

    private void makeredpose (){

    //
    }
}
