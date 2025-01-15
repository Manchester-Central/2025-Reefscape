// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.chaos131.poses.MirroredDrivePose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class FieldPoint {
     /** The pre-calculated red pose */
  protected final Pose2d m_redPose;

  /** The pre-calculated blue pose */
  protected final Pose2d m_bluePose;

  /** A potentially null name for the pose */
  protected final String m_name;

  /** The corresponding alliance the original pose was built for */
  protected final Alliance m_defaultAlliance;

  protected final double fieldLength = 17.524;

  protected final double fieldWidth = 8.052;
    public static final FieldPoint processor = new FieldPoint("processor", new Pose2d(5.988,0,Rotation2d.fromDegrees(90)));
    public static final FieldPoint leftSource = new FieldPoint("leftSource", new Pose2d(16.685,7.398,Rotation2d.fromDegrees(90)));
    public static final FieldPoint rightSource = new FieldPoint("rightSource", new Pose2d(16.685,0.6553,Rotation2d.fromDegrees(90)));

    public FieldPoint (String name, Pose2d pose) {
        m_name = name;
        m_bluePose = pose;
        m_defaultAlliance = Alliance.Blue;
        Translation2d pTranslation2d = pose.getTranslation().minus(new Translation2d(fieldLength, fieldWidth).div(2));
        pTranslation2d = pTranslation2d.rotateBy(Rotation2d.fromDegrees(180));
        pTranslation2d = pTranslation2d.plus(new Translation2d(fieldLength, fieldWidth).div(2));
        m_redPose = new Pose2d(pTranslation2d, pose.getRotation().plus(Rotation2d.fromDegrees(180)));

    }

    public Pose2d getBluePose() {

        return m_bluePose;
    }

    public Pose2d getRedPose() {

        return m_redPose;
    }
}
