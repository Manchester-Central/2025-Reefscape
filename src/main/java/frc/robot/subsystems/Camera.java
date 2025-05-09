// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chaos131.vision.CameraSpecs;
import com.chaos131.vision.LimelightCamera;
import com.chaos131.vision.VisionData;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableValue;
import frc.robot.LimelightHelpers;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Our implementation of a LimeLightCamera.
 */
public class Camera extends LimelightCamera {
  /** Creates a new FrontCamera. */
  public Camera(
      String name,
      LimelightVersion limelightVersion,
      CameraSpecs specs,
      Supplier<Pose2d> poseSupplier,
      Consumer<VisionData> poseConsumer,
      Supplier<Double> robotSpeedSupplier,
      Supplier<Double> robotRotationSpeedSupplier) {
    super(
        name,
        limelightVersion,
        specs,
        poseSupplier,
        poseConsumer,
        robotSpeedSupplier,
        robotRotationSpeedSupplier);

    m_botpose.readQueue();
    m_botposeMT2.readQueue();
  }

  public Pose3d getTargetPose3dRobotSpace() {
    return LimelightHelpers.getTargetPose3d_RobotSpace(m_name);
  }

  public Pose3d getBotPose3dTargetSpace() {
    return LimelightHelpers.getBotPose3d_TargetSpace(m_name);
  }

  @Override
  public void periodic() {
    LoadNTQueueToVisionData();

    /*
     * This step will replace all that data (ie, no data) if we're in replay mode by
     * reading values.
     * from the log file. If we're a real robot, we instead log the data TO the
     * file!
     */
    Logger.processInputs(m_name, m_poseData);

    // Now we do that thang with the all the data we received.
    processUpdateQueue();

    // If the timer has expired, so set the state to inactive...
    if (m_poseData.timestamps.length > 0 || m_targetData.timestamps.length > 0) {
      m_disconnectedTimer.reset();
      m_activeData = true;
    } else {
      m_activeData = false;
    }
    Logger.recordOutput(m_name + "/ActiveData", m_activeData);
    Logger.recordOutput(
        m_name + "/DisconnectTimer",
        m_disconnectedTimer.get() < m_timerMinimum ? 0.0 : m_disconnectedTimer.get());
  }

  @Override
  public VisionData processMeasuredData(int idx) {
    if (m_poseData.timestamps.length <= idx) {
      return null;
    }

    // MT1
    var conf =
        calculateConfidence(
            m_poseData.pose[idx],
            m_poseData.tagCount[idx],
            m_poseData.averageTagDistance[idx],
            m_poseData.deviations[0]);

    return new VisionData(
        m_poseData.pose[idx],
        m_poseData.timestamps[idx],
        new double[] {m_poseData.deviations[idx], m_poseData.deviations[idx], 1},
        conf, m_name);
  }

  @Override
  protected void LoadNTQueueToVisionData() {
    /*
     * TODO: Serious race condition concern with MT1 and MT2! I can't simply fix this, Limelight needs to.
     */
    NetworkTableValue[] mt1Poses = m_botpose.readQueue();
    Logger.recordOutput(m_name + "/DataLength", mt1Poses.length);
    @SuppressWarnings("unused")
    NetworkTableValue[] mt2Poses = m_botposeMT2.readQueue();
    m_poseData.reset();
    // TODO: Investigate TableEntry issues...
    if (mt1Poses.length == 1 && mt1Poses[0] == null) {
      return;
    }
    if (mt1Poses.length == 0) {
      return;
    }

    // Parse MegaTag1 Info
    m_poseData.resize(mt1Poses.length);
    for (int idx = 0; idx < mt1Poses.length; idx++) {
      long timestamp = mt1Poses[idx].getTime();
      var data = mt1Poses[idx].getDoubleArray();

      final double timestampSeconds = timestamp / 1000000.0 - data[idxLatency] / 1000.0;

      var posePosition = new Translation3d(data[idxX], data[idxY], data[idxZ]);

      var poseRotation = new Rotation3d(
          data[idxRoll] * Math.PI / 180,
          data[idxPitch] * Math.PI / 180,
          data[idxYaw] * Math.PI / 180);

      var visionPose = new Pose3d(posePosition, poseRotation);

      if (m_offset != null) {
        var cameraOffset = m_offset.get();
        cameraOffset = cameraOffset.rotateBy(new Rotation3d(0, 0, data[idxYaw] * Math.PI / 180));
        cameraOffset.getRotation().getZ();
        visionPose = new Pose3d(
            new Translation3d(
                visionPose.getX() - cameraOffset.getX(),
                visionPose.getY() - cameraOffset.getY(),
                visionPose.getZ() - cameraOffset.getZ()),
            new Rotation3d(
                0, // poseRotation.getX() - cameraOffset.getRotation().getX(),
                0, // poseRotation.getY() - cameraOffset.getRotation().getY(),
                poseRotation.getZ()) // - cameraOffset.getRotation().getZ())
        );
      }

      double[] deviation = calculateTranslationalDeviations(data[idxTagDistance], data[idxTagCount]);

      m_poseData.averageTagDistance[idx] = data[idxTagDistance];
      m_poseData.deviations[idx] = deviation[0];
      m_poseData.pose[idx] = visionPose;
      m_poseData.tagCount[idx] = (int) data[idxTagCount];
      m_poseData.timestamps[idx] = timestampSeconds;
    } // End MT1

    // silly :3
  }
}
