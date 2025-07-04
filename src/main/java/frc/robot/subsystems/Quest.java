// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import com.chaos131.vision.VisionData;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Time;
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
    Pose2d questPose = questNav.getPose();
    Pose2d robotPose = questPose.transformBy(QuestNavConstants.RobotToQuest.inverse());
    Pose3d robotPose3d = new Pose3d(robotPose);
    Logger.recordOutput("Quest/isConnected", questNav.isConnected());
    Logger.recordOutput("Quest/isTracking", questNav.isTracking());
    Logger.recordOutput("Quest/questPose", questPose);
    Logger.recordOutput("Quest/robotPose", robotPose);
    Logger.recordOutput("Quest/battery", questNav.getBatteryPercent());
    Logger.recordOutput("Quest/robotPose3d", robotPose3d);
    if (DriverStation.isEnabled()) {
      // Matrix<N3, N1> QUESTNAV_STD_DEVS =
      //     VecBuilder.fill(
      //         0.02, // Trust down to 2cm in X direction
      //         0.02, // Trust down to 2cm in Y direction
      //         0.035 // Trust down to 2 degrees rotational
      //     );

      if (questNav.isConnected() && questNav.isTracking()) {
        // Get timestamp from the QuestNav instance
        Time timestamp = Microseconds.of(questNav.getDataTimestamp());
        // double ctreTimestamp = Utils.fpgaToCurrentTime(timestamp);

        // You can put some sort of filtering here if you would like!

        // Add the measurement to our estimator
        m_swerveDrive.addVisionMeasurement(new VisionData(robotPose3d, timestamp.in(Seconds), new double[] {0.02, 0.02, 0.035}, 1, getName())); //TODO Find a better way to get a Pose3d value.
        // m_swerveDrive.resetPose(robotPose);
      }
    } else {
      Pose2d robotDisabledPose = m_swerveDrive.getPose();
      Pose2d questDisabledPose = robotDisabledPose.transformBy(QuestNavConstants.RobotToQuest);
      questNav.setPose(questDisabledPose);
    }
  }
}
