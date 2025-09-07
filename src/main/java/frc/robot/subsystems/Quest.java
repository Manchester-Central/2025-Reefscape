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
  private boolean isResetting = false;
  private boolean isResetActive = false;
  private boolean planB = false;

  /** Creates a new Quest. */
  public Quest(SwerveDrive swerveDrive) {
    m_swerveDrive = swerveDrive;
  }

  Pose2d questPose = null;
  Pose2d robotPose = null;

  @Override
  public void periodic() {
    questNav.commandPeriodic();
    Transform2d robotToQuest = new Transform2d(
      Inches.of(QuestNavConstants.RobotToQuestXInches.get()), 
      Inches.of(QuestNavConstants.RobotToQuestYInches.get()), 
      QuestNavConstants.RobotToQuestRotation);
    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
    OptionalInt battery = questNav.getBatteryPercent();
    Logger.recordOutput("Quest/isConnected", questNav.isConnected());
    Logger.recordOutput("Quest/isTracking", questNav.isTracking());
    Logger.recordOutput("Quest/battery", battery.isPresent() ? battery.getAsInt() : 0);
    Logger.recordOutput("Quest/isResetting", isResetting);
    Logger.recordOutput("Quest/isResetActive", isResetActive);
    Logger.recordOutput("Quest/planB", planB);
    if (poseFrames.length > 0) {
      questPose = poseFrames[poseFrames.length - 1].questPose();
      robotPose = questPose.transformBy(robotToQuest.inverse());
    }
    if (robotPose == null) {
      return;
    }
    Pose3d robotPose3d = new Pose3d(robotPose);
    Logger.recordOutput("Quest/questPose", questPose);
    Logger.recordOutput("Quest/robotPose", robotPose);
    Logger.recordOutput("Quest/robotPose3d", robotPose3d);
    planB = DriverStation.isEnabled() && !isResetting;
    if (planB) {
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

          // Add the measurement to our estimator
          m_swerveDrive.addVisionMeasurement(new VisionData(robotPose3d, timestamp, QUESTNAV_STD_DEVS.getData(), 1, getName())); //TODO Find a better way to get a Pose3d value.
          // m_swerveDrive.resetPose(robotPose);

          // Add the measurement to our estimator

        }
      } 
    } else {
      isResetActive = true;
      Pose2d robotDisabledPose = m_swerveDrive.getPose();
      Pose2d questDisabledPose = robotDisabledPose.transformBy(robotToQuest);
      questNav.setPose(questDisabledPose);
    }
  }

  public void resetQuestPose(boolean resetting) {
    isResetting = resetting;
  }
}

