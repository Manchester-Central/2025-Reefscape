// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.chaos131.robot.ChaosRobot;
import com.chaos131.util.DashboardNumber;
import com.chaos131.util.FieldData;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.arm.Gripper;
import frc.robot.utils.FieldPoint;
import frc.robot.utils.LocalADStarAK;
import java.util.ArrayList;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends ChaosRobot {

  /**
   * Creates logging values from out BuildConstants.
   */
  protected void setupRobot() {
    Logger.recordMetadata("RobotHash", BuildConstants.GIT_SHA);
    Logger.recordMetadata("RobotBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("RobotDirty", (BuildConstants.DIRTY == 1) ? "Yes" : "No");
    Logger.recordMetadata("LastBuild", BuildConstants.BUILD_DATE);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() throws Exception {
    super(GeneralConstants.RobotMode);
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();
    SimulatedArena.getInstance().simulationPeriodic();
    Pose3d[] coralPose = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
    Logger.recordOutput("Field/Piece", coralPose);
    Logger.recordOutput(
        "Field/Reef Apriltags", FieldData.GatherAprilTagPoses(FieldPoint.blueReefAprilTags()));
    ArrayList<FieldPoint> reefSwervePoses = FieldPoint.getReefDrivePoses();
    Pose2d[] reefPositions = new Pose2d[reefSwervePoses.size()];
    for (int i = 0; i < reefPositions.length; i++) {
      reefPositions[i] = reefSwervePoses.get(i).getCurrentAlliancePose();
    }
    Logger.recordOutput("Field/ReefPositions", reefPositions);
    // System.out.println(coralPose.length);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DashboardNumber.checkAll();
    ((RobotContainer) m_robotContainer).setSwerveDriveAcceptingVisionUpdates(isDisabled() ? true : SwerveConstants.AcceptVisionUpdates);
  }

  @Override
  public void robotInit() {
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathfindingCommand.warmupCommand().schedule();
    Gripper.hasCoralGrippedSim = true;
    ((RobotContainer) m_robotContainer).setMotorCleanUp();
    super.robotInit(); 
  }

  @Override
  public void disabledCleanup() {
    ((RobotContainer) m_robotContainer).setMotorCleanUp();
  }

  @Override
  public void teleopInit() {
    ((RobotContainer) m_robotContainer).setMotorStartUp();
    ((RobotContainer) m_robotContainer).autoAndTeleInit();
    super.teleopInit();
  }

  @Override
  public void autonomousInit() {
    ((RobotContainer) m_robotContainer).setMotorStartUp();
    ((RobotContainer) m_robotContainer).autoAndTeleInit();
    Gripper.hasCoralGrippedSim = true;
    super.autonomousInit();
  }
}
