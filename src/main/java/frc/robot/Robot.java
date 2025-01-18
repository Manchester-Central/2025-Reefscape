// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.chaos131.robot.ChaosRobot;
import org.littletonrobotics.junction.Logger;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends ChaosRobot {
  protected void setupRobot() {
    Logger.recordMetadata("RobotHash", BuildConstants.GIT_SHA);
    Logger.recordMetadata("RobotBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("RobotDirty", (BuildConstants.DIRTY == 1) ? "Yes" : "No");
    Logger.recordMetadata("LastBuild", BuildConstants.BUILD_DATE);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   *
   * @throws Exception
   */
  public Robot() throws Exception {
    super(Mode.SIM);
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }
}
