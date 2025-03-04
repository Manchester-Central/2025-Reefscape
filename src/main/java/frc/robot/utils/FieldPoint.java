package frc.robot.utils;

import com.chaos131.util.FieldData;
import com.chaos131.vision.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldDimensions;
import frc.robot.Constants.RobotDimensions;
import java.util.ArrayList;
import java.util.HashMap;

import org.opencv.core.Point;

/**
 * A class to help managing positions on the field (for either alliance color).
 */
public class FieldPoint {
  /** The pre-calculated red pose. */
  protected final Pose2d m_redPose;

  /** The pre-calculated blue pose. */
  protected final Pose2d m_bluePose;

  /** A potentially null name for the pose. */
  protected final String m_name;

  /** The corresponding alliance the original pose was built for. */
  protected final Alliance m_defaultAlliance;

  protected final double m_fieldLength = FieldDimensions.FieldLength;
  protected final double m_fieldWidth = FieldDimensions.FieldWidth;

  // List of named points on the field
  public static final FieldPoint processor = new FieldPoint("processor",
      new Pose2d(5.988, 0, Rotation2d.fromDegrees(90)));
  public static final FieldPoint leftSource = new FieldPoint("leftSource",
      new Pose2d(0.8512, 7.396, Rotation2d.fromDegrees(90)));
  public static final FieldPoint rightSource = new FieldPoint("rightSource",
      new Pose2d(0.852, 0.6553, Rotation2d.fromDegrees(90)));
  public static final FieldPoint testPoint = new FieldPoint("testPoint",
      new Pose2d(10.0, 5.0, Rotation2d.fromDegrees(37)));
      

  public static HashMap<Integer, AprilTag> aprilTagMap = FieldData.GetAprilTagMap("assets/frc2025.fmap");

  public static final FieldPoint ReefPose2 = new FieldPoint("reefPose2", aprilTagMap.get(22).pose2d);
  public static final FieldPoint ReefPose4 = new FieldPoint("reefPose4", aprilTagMap.get(17).pose2d);
  public static final FieldPoint ReefPose6 = new FieldPoint("reefPose6", aprilTagMap.get(18).pose2d);
  public static final FieldPoint ReefPose8 = new FieldPoint("reefPose8", aprilTagMap.get(19).pose2d);
  public static final FieldPoint ReefPose10 = new FieldPoint("reefPose10", aprilTagMap.get(20).pose2d);
  public static final FieldPoint ReefPose12 = new FieldPoint("reefPose12", aprilTagMap.get(21).pose2d);

  /**
   * Gets the april tabs for the blue reef.
   */
  public static ArrayList<AprilTag> blueReefAprilTags() {
    ArrayList<AprilTag> blueTagArrayList = new ArrayList<AprilTag>();
    blueTagArrayList.add(aprilTagMap.get(17));
    blueTagArrayList.add(aprilTagMap.get(18));
    blueTagArrayList.add(aprilTagMap.get(19));
    blueTagArrayList.add(aprilTagMap.get(20));
    blueTagArrayList.add(aprilTagMap.get(21));
    blueTagArrayList.add(aprilTagMap.get(22));
    return blueTagArrayList;
  }

  /**
   * Gets the april tags for the blue HP positions.
   */
  public static ArrayList<AprilTag> blueHpAprilTags() {
    ArrayList<AprilTag> blueTagArrayList = new ArrayList<AprilTag>();
    blueTagArrayList.add(aprilTagMap.get(12));
    blueTagArrayList.add(aprilTagMap.get(13));
    return blueTagArrayList;
  }

  /**
   * Gets all the drive positions we can consider for scoring on the reef (left
   * and right of each april tag).
   */
  public static ArrayList<FieldPoint> getReefDrivePoses() {
    ArrayList<FieldPoint> reefDrivePoses = new ArrayList<FieldPoint>();
    for (AprilTag aprilTag : blueReefAprilTags()) {
      Pose2d leftPose = aprilTag.pose2d.transformBy(
          new Transform2d(
              RobotDimensions.FrontBackLengthMeters / 2 + RobotDimensions.RobotToReefMargin,
              FieldDimensions.ReefBranchLeft.getY(),
              Rotation2d.fromDegrees(180)));
      reefDrivePoses.add(new FieldPoint(aprilTag.id + " ReefLeft", leftPose));
      Pose2d rightPose = aprilTag.pose2d.transformBy(
          new Transform2d(
              RobotDimensions.FrontBackLengthMeters / 2 + RobotDimensions.RobotToReefMargin,
              FieldDimensions.ReefBranchRight.getY(),
              Rotation2d.fromDegrees(180)));
      reefDrivePoses.add(new FieldPoint(aprilTag.id + " ReefRight", rightPose));
    }
    return reefDrivePoses;
  }

  /**
   * Returns an array list of all the april tags on the reef is made.
   */
  public static ArrayList<FieldPoint> getReefAprilTagPoses() {
    ArrayList<FieldPoint> reefAprilTagPoses = new ArrayList<FieldPoint>();
    for (AprilTag aprilTag : blueReefAprilTags()) {
      reefAprilTagPoses.add(new FieldPoint(aprilTag.id + " ReefTag", aprilTag.pose2d));
    }
    return reefAprilTagPoses;
  }

  /**
   * Gets our positions for picking up coral from the HP stations.
   */
  public static ArrayList<FieldPoint> getHpDrivePoses() {
    ArrayList<FieldPoint> hpDrivePoses = new ArrayList<FieldPoint>();
    for (AprilTag aprilTag : blueHpAprilTags()) {
      Pose2d leftPose = aprilTag.pose2d.transformBy(
          new Transform2d(
              RobotDimensions.FrontBackLengthMeters / 2 + 0.05,
              RobotDimensions.SideSideLengthMeters / 2 + 0.05,
              Rotation2d.fromDegrees(180)));
      hpDrivePoses.add(new FieldPoint(aprilTag.id + " HPLeft", leftPose));
      Pose2d rightPose = aprilTag.pose2d.transformBy(
          new Transform2d(
              RobotDimensions.FrontBackLengthMeters / 2 + 0.05,
              -RobotDimensions.SideSideLengthMeters / 2 - 0.05,
              Rotation2d.fromDegrees(180)));
      hpDrivePoses.add(new FieldPoint(aprilTag.id + " HPRight", rightPose));
    }
    return hpDrivePoses;
  }

  public static FieldPoint getNearestPoint(Pose2d RobotPose, ArrayList<FieldPoint> Points) {
    FieldPoint Nearest = null;
    double minimum_distance = 99;
    for (FieldPoint PT : Points) {
      double distance = RobotPose.relativeTo(PT.getCurrentAlliancePose()).getTranslation().getNorm();
      if (distance < minimum_distance) {
        minimum_distance = distance;
        Nearest = PT;
      }
    }
    return Nearest;
  }

  /**
   * Creates a new FieldPoint.
   *
   * @param name the name of the field point
   * @param pose the blue alliance pose on the field
   */
  public FieldPoint(String name, Pose2d pose) {
    m_name = name;
    m_bluePose = pose;
    m_defaultAlliance = Alliance.Blue;
    Translation2d poseTranslation = pose.getTranslation().minus(new Translation2d(m_fieldLength, m_fieldWidth).div(2));
    poseTranslation = poseTranslation.rotateBy(Rotation2d.fromDegrees(180));
    poseTranslation = poseTranslation.plus(new Translation2d(m_fieldLength, m_fieldWidth).div(2));
    m_redPose = new Pose2d(poseTranslation, pose.getRotation().plus(Rotation2d.fromDegrees(180)));
  }

  public Pose2d getBluePose() {
    return m_bluePose;
  }

  public Pose2d getRedPose() {
    return m_redPose;
  }

  public String getName() {
    return m_name;
  }

  /**
   * Gets the appropriate pose for the current alliance color. (Override this when
   * using unit
   * testing since the call to DriverStation will throw an exception)
   */
  protected Alliance getCurrentAlliance() {
    return DriverStation.getAlliance().orElse(m_defaultAlliance);
  }

  /**
   * Gets the pose for the robot's current alliance. (If the DS is not connected,
   * the default
   * alliance is used)
   */
  public Pose2d getCurrentAlliancePose() {
    return getCurrentAlliance() == Alliance.Blue ? m_bluePose : m_redPose;
  }
}
