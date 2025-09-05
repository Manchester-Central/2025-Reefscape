// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.OptionalInt;

import org.littletonrobotics.junction.Logger;

import com.chaos131.vision.VisionData;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import gg.questnav.questnav.PoseFrame;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.QuestNavConstants;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;


public class Quest extends SubsystemBase {
  QuestNav questNav = new QuestNav();
  private SwerveDrive m_swerveDrive;

  /** Creates a new Quest. */
  public Quest(SwerveDrive swerveDrive) {
    m_swerveDrive = swerveDrive;
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic();
    OptionalInt battery = questNav.getBatteryPercent();
    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
    if (poseFrames.length <= 0) {
      return;
    }
    Transform2d robotToQuest = new Transform2d(
        Inches.of(QuestNavConstants.RobotToQuestXInches.get()), 
        Inches.of(QuestNavConstants.RobotToQuestYInches.get()), 
        QuestNavConstants.RobotToQuestRotation);
    Pose2d questPose = poseFrames[poseFrames.length - 1].questPose();
    Pose2d robotPose = questPose.transformBy(robotToQuest.inverse());
    Pose3d robotPose3d = new Pose3d(robotPose);
    Logger.recordOutput("Quest/isConnected", questNav.isConnected());
    Logger.recordOutput("Quest/isTracking", questNav.isTracking());
    Logger.recordOutput("Quest/questPose", questPose);
    Logger.recordOutput("Quest/robotPose", robotPose);
    Logger.recordOutput("Quest/battery", battery.isPresent() ? battery.getAsInt() : 0);
    Logger.recordOutput("Quest/robotPose3d", robotPose3d);
    if (DriverStation.isEnabled()) {
      Matrix<N3, N1> QUESTNAV_STD_DEVS =
          VecBuilder.fill(
              0.02, // Trust down to 2cm in X direction
              0.02, // Trust down to 2cm in Y direction
              0.035 // Trust down to 2 degrees rotational
          );

      if (questNav.isConnected() && questNav.isTracking()) {
        // Loop over the pose data frames and send them to the pose estimator
        for (PoseFrame questFrame : poseFrames) {
          // Get the pose of the Quest
          Pose3d questPose3d = new Pose3d(questFrame.questPose());
          // Get timestamp for when the data was sent
          double timestamp = questFrame.dataTimestamp();

          // Transform by the mount pose to get your robot pose
          robotPose3d = questPose3d.transformBy(new Transform3d(robotToQuest.inverse()));

          // Convert FPGA timestamp to CTRE's time domain using Phoenix 6 utility
          double ctreTimestamp = Utils.fpgaToCurrentTime(timestamp);


          // Add the measurement to our estimator
          m_swerveDrive.addVisionMeasurement(new VisionData(robotPose3d, ctreTimestamp, new double[] {0.02, 0.02, 0.035}, 1, getName())); //TODO Find a better way to get a Pose3d value.
          // m_swerveDrive.resetPose(robotPose);

          // Add the measurement to our estimator

        }
      } 
    } else {
      Pose2d robotDisabledPose = m_swerveDrive.getPose();
      Pose2d questDisabledPose = robotDisabledPose.transformBy(robotToQuest);
      questNav.setPose(questDisabledPose);
    }
  }
}

