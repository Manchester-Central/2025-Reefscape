// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chaos131.vision.VisionData;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.QuestNavConstants;
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
    if (DriverStation.isEnabled()) {
      Pose2d questPose = questNav.getPose();
      Pose2d robotPose = questPose.transformBy(QuestNavConstants.RobotToQuest.inverse());
      Matrix<N3, N1> QUESTNAV_STD_DEVS =
          VecBuilder.fill(
              0.02, // Trust down to 2cm in X direction
              0.02, // Trust down to 2cm in Y direction
              0.035 // Trust down to 2 degrees rotational
          );

      if (questNav.isConnected() && questNav.isTracking()) {
        // Get timestamp from the QuestNav instance
        double timestamp = questNav.getDataTimestamp();

        // You can put some sort of filtering here if you would like!

        // Add the measurement to our estimator
        m_swerveDrive.addVisionMeasurement(new VisionData(new Pose3d(robotPose), timestamp, QUESTNAV_STD_DEVS.getData(), timestamp, getName())); //TODO Find a better way to get a Pose3d value.
      }
    } else {
      Pose2d robotPose = m_swerveDrive.getPose();
      Pose2d questPose = robotPose.transformBy(QuestNavConstants.RobotToQuest);
      questNav.setPose(questPose);
    }
  }
}
