// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotDimensions;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmValues;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotation;

import java.util.HashMap;
import java.util.LinkedList;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;



/** A class for sending a 2d representation of our robot over network tables. */
public class MechManager2D extends SubsystemBase {
  private Arm m_arm;

  @AutoLogOutput(key = "Mech2d/Arm")
  private LoggedMechanism2d m_armBase;
  private HashMap<String, LoggedMechanismLigament2d> m_ligaments;
  private HashMap<LoggedMechanismLigament2d, LoggedMechanismLigament2d> m_parentMap;

  // Extender
  private LoggedMechanismRoot2d m_armRoot;
  private LoggedMechanismLigament2d m_extenderLigament;
  // private LoggedMechanismLigament2d m_extenderBaseLigament;

  // Gripper
  private LoggedMechanismLigament2d m_gripperBaseLigament;
  private LoggedMechanismLigament2d m_gripperBaseVericalLigament;
  private LoggedMechanismLigament2d m_gripperWristLigament;
  private LoggedMechanismLigament2d m_gripperCenterLigament;
  private LoggedMechanismLigament2d m_gripperMidSupportLigament;
  private LoggedMechanismLigament2d m_gripperMidWheelsLigament;
  private LoggedMechanismLigament2d m_gripperAlgaeSupportLigament;
  @SuppressWarnings("unused")
  private LoggedMechanismLigament2d m_gripperAlgaeWheelsLigament;
  private LoggedMechanismLigament2d m_gripperCoralWheelsLigament;
  private LoggedMechanismLigament2d m_gripperCoralFrontLigament;
  private LoggedMechanismLigament2d m_gripperCoralBackLigament;


  private final Color8Bit m_extenderColor = new Color8Bit(0, 0, 255);
  private final Color8Bit m_gripperNeutralColor = new Color8Bit(100, 100, 100);
  private final Color8Bit m_gripperForwardColor = new Color8Bit(0, 255, 0);
  private final Color8Bit m_gripperReverseColor = new Color8Bit(255, 0, 0);
  private final Color8Bit m_gripperHasCoralColor = new Color8Bit(255, 255, 255);
  private final Color8Bit m_gripperHasAlgaeColor = new Color8Bit(0, 128, 128);

  /**
   * Creates a new mech manager.
   */
  public MechManager2D(Arm arm) {
    m_arm = arm;
    m_ligaments = new HashMap<>();
    m_parentMap = new HashMap<>();

    m_armBase = new LoggedMechanism2d(0,0);
    m_armRoot = m_armBase.getRoot("Arm", RobotDimensions.BasePivotOffset.getX(), RobotDimensions.BasePivotOffset.getY());
    m_extenderLigament = m_armRoot.append(makeLigament(null, // Note that this one is special because it's on the root
                                                        "Extender",
                                                        Meters.of(0.001),
                                                        Degrees.of(90),
                                                        6, m_extenderColor));

    /* Gripper */
    m_gripperBaseLigament = makeLigament(m_extenderLigament,
                                        "GripperBase",
                                        Meters.of(RobotDimensions.ArmToWristOffset.getTranslation().getNorm()),
                                        RobotDimensions.WristMountAngle.getMeasure(),
                                        4, m_gripperNeutralColor);
    // Reorients our gripper coordinate frame to point back up 
    m_gripperBaseVericalLigament = makeLigament(m_gripperBaseLigament,
                                                "GripperBaseVertical",
                                                Meters.of(0.0001),
                                                RobotDimensions.WristMountAngle.getMeasure().times(-1),
                                                1, m_gripperNeutralColor);
    m_gripperWristLigament = makeLigament(m_gripperBaseVericalLigament,
                                          "GripperWrist",
                                          Meters.of(0.001),
                                          Degrees.of(0),
                                          2, m_gripperNeutralColor);
    m_gripperCenterLigament = makeLigament(m_gripperWristLigament,
                                          "GripperCenter",
                                          Meters.of(RobotDimensions.WristToCoralIntakeAxle),
                                          Degrees.of(0),
                                          2, m_gripperNeutralColor);
    
    // Gripper Supports and Parts
    m_gripperMidSupportLigament = makeLigament(m_gripperCenterLigament,
                                                "GripperMidSupport",
                                                Meters.of(0.05),
                                                Degrees.of(-45),
                                                2, m_gripperNeutralColor);
    m_gripperMidWheelsLigament = makeLigament(m_gripperMidSupportLigament,
                                              "GripperMidWheels",
                                              Meters.of(0.05),
                                              Degrees.of(0),
                                              2, m_gripperHasCoralColor);
    m_gripperAlgaeSupportLigament = makeLigament(m_gripperCenterLigament,
                                                  "GripperAlgaeSupport",
                                                  Meters.of(0.2),
                                                  Degrees.of(64),
                                                  2, m_gripperNeutralColor);
    m_gripperAlgaeWheelsLigament = makeLigament(m_gripperAlgaeSupportLigament,
                                                "GripperAlgaeWheels",
                                                Meters.of(0.05),
                                                Degrees.of(0),
                                                2, m_gripperHasAlgaeColor);
    m_gripperCoralWheelsLigament = makeLigament(m_gripperCenterLigament,
                                                "CoralWheels",
                                                Meters.of(RobotDimensions.WristToCoralIntakeAxle),
                                                Degrees.of(-90),
                                                2, m_gripperNeutralColor);
    m_gripperCoralFrontLigament = makeLigament(m_gripperCoralWheelsLigament,
                                          "GripperCoralFront",
                                          Meters.of(0.0001),
                                          Degrees.of(90),
                                          10, m_gripperHasCoralColor);
    m_gripperCoralBackLigament = makeLigament(m_gripperCoralWheelsLigament,
                                              "GripperCoralBack",
                                              Meters.of(0.0001),
                                              Degrees.of(-90),
                                              10, m_gripperHasCoralColor);
  }

  public Pose2d calculateEndPoint(String ligamentName) {
    LoggedMechanismLigament2d ligament = m_ligaments.get(ligamentName);
    LinkedList<LoggedMechanismLigament2d> sequence = new LinkedList<>();
    sequence.addFirst(ligament);

    var current = ligament;
    while (current != null) {
      var parent = m_parentMap.get(ligament);
      sequence.addFirst(parent);
      current = parent;
    }

    // Now, we calculate the position!
    sequence.pop(); // Discard the first element (it's null)
    Pose2d currentPose = new Pose2d(RobotDimensions.BasePivotOffset.getX(), RobotDimensions.BasePivotOffset.getY(), Rotation2d.kZero);
    for (var lig : sequence) {
      var rotatedTranslation = new Transform2d(Meters.of(lig.getLength()), Meters.of(0), Rotation2d.fromDegrees(lig.getAngle()));
      currentPose = currentPose.transformBy(rotatedTranslation);
    }

    return currentPose;
  }

  public Pose3d calculateEndPose3d(Pose2d robotPose, String ligamentName) {
    Pose2d mechanismPoint = calculateEndPoint(ligamentName);
    Pose2d fieldRelativeGroundPoint = robotPose.transformBy(new Transform2d(mechanismPoint.getX(), 0, Rotation2d.kZero));
    return new Pose3d(fieldRelativeGroundPoint.getX(), fieldRelativeGroundPoint.getY(), mechanismPoint.getY(),
                      new Rotation3d(0, -mechanismPoint.getRotation().getRadians(), 0));
  }

  private LoggedMechanismLigament2d makeLigament(LoggedMechanismLigament2d parent, String name, Distance length, Angle angle, double lineWidth, Color8Bit color) {
    var ligament = new LoggedMechanismLigament2d(name, length.in(Meters), angle.in(Degrees), lineWidth, color);
    m_ligaments.put(name, ligament);
    if (parent != null) parent.append(ligament);
    m_parentMap.put(ligament, parent);
    return ligament;
  }

  @Override
  public void periodic() {
    ArmValues values = m_arm.getArmValues();

    // Set angles and length of Arm parts
    m_extenderLigament.setLength(values.extenderLength == 0 ? 0.0001 : values.extenderLength);
    m_extenderLigament.setAngle(values.basePivotAngle);
    m_gripperWristLigament.setAngle(values.gripperPivotAngle);

    // Change coral gripper color
    if (values.coralGripSpeed == 0) {
      m_gripperCoralWheelsLigament.setColor(m_gripperNeutralColor);
    } else if (values.coralGripSpeed > 0) {
      m_gripperCoralWheelsLigament.setColor(m_gripperForwardColor);
    } else {
      m_gripperCoralWheelsLigament.setColor(m_gripperReverseColor);
    }

    // Change color if holding a coral
    if (values.hasCoral) {
      m_gripperCoralFrontLigament.setLength(RobotDimensions.WristToCoralFront.getX() - RobotDimensions.WristToCoralIntakeAxle);
      m_gripperCoralBackLigament.setLength(RobotDimensions.WristToCoralIntakeAxle - RobotDimensions.WristToCoralBack.getX());
    } else {
      m_gripperCoralFrontLigament.setLength(0.001);
      m_gripperCoralBackLigament.setLength(0.001);
    }
  }
}
