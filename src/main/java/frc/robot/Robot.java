// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.chaos131.robot.ChaosRobot;
import com.chaos131.util.DashboardNumber;
import com.chaos131.util.FieldData;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.arm.Gripper;
import frc.robot.utils.FieldPoint;
import frc.robot.utils.LocalADStarAK;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends ChaosRobot {

  public String m_autoName = "";
  public String m_newAutoName = "";

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
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
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

  @Override
  public void disabledPeriodic() {
    m_newAutoName = m_robotContainer.getAutonomousCommand().getName();
    super.disabledPeriodic();
    if (m_autoName != m_newAutoName) {
      m_autoName = m_newAutoName;
      if (AutoBuilder.getAllAutoNames().contains(m_autoName)) {
        System.out.println("Displaying " + m_autoName);
        List<PathPlannerPath> pathPlannerPaths;
        try {
          pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(m_autoName);
        } catch (Exception e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
          return;
        }
        List<Pose2d> poses = new ArrayList<>();
        for (PathPlannerPath path : pathPlannerPaths) {
          poses.addAll(path.getAllPathPoints().stream().map(point -> new Pose2d(point.position.getX(), point.position.getY(), new Rotation2d())).collect(Collectors.toList()));
        }
        getRobotContainer().getField().getObject("path").setPoses(poses);
      }
      else {
        getRobotContainer().getField().getObject("path").setPoses(new ArrayList<>());
      }
    }

  }

  public RobotContainer getRobotContainer() {
    return (RobotContainer) m_robotContainer;
  }
}
