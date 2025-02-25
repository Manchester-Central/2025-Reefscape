// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotDimensions;
import frc.robot.subsystems.lift.IdLift;
import frc.robot.subsystems.lift.IdLift.IdLiftValues;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** A class for sending a 2d representation of our robot over network tables. */
public class MechManager2D extends SubsystemBase {
  private IdLift m_idLift;

  @AutoLogOutput(key = "Mech2d/Lift")
  private LoggedMechanism2d m_liftBase;

  // Extender
  private LoggedMechanismRoot2d m_liftRoot;
  private LoggedMechanismLigament2d m_extenderLigament;
  private LoggedMechanismLigament2d m_extenderBaseLigament;

  // Gripper
  @SuppressWarnings("unused")
  private LoggedMechanismLigament2d m_gripperBaseLigament;
  @SuppressWarnings("unused")
  private LoggedMechanismLigament2d m_gripperBaseVericalLigament;
  private LoggedMechanismLigament2d m_gripperPivotLigament;
  private LoggedMechanismLigament2d m_gripperBodyLigament;
  private LoggedMechanismLigament2d m_gripperCoralLigament;

  // Bucket
  private LoggedMechanismLigament2d m_bucketBaseLigament;
  private LoggedMechanismLigament2d m_bucketFlatLowerLigament;
  @SuppressWarnings("unused")
  private LoggedMechanismLigament2d m_bucketFlatUpperLigament;
  private LoggedMechanismLigament2d m_bucketVerticalLowerLigament;
  @SuppressWarnings("unused")
  private LoggedMechanismLigament2d m_bucketVerticalUpperLigament;
  @SuppressWarnings("unused")
  private LoggedMechanismLigament2d m_bucketSlotLigament;

  private final Color8Bit m_extenderColor = new Color8Bit(0, 0, 255);
  private final Color8Bit m_gripperNeutralColor = new Color8Bit(100, 100, 100);
  private final Color8Bit m_gripperForwardColor = new Color8Bit(0, 255, 0);
  private final Color8Bit m_gripperReverseColor = new Color8Bit(255, 0, 0);
  private final Color8Bit m_gripperHasCoralColor = new Color8Bit(255, 255, 255);
  private final Color8Bit m_bucketColor = new Color8Bit(150, 150, 150);
  private final Color8Bit m_bucketPassableColor = new Color8Bit(200, 200, 200);

  /**
   * Creates a new mech manager.
   */
  public MechManager2D(IdLift idLift) {
    m_idLift = idLift;

    m_liftBase = new LoggedMechanism2d(0,0);
    
    m_liftRoot = m_liftBase.getRoot("Lift", RobotDimensions.BasePivotOffset.getTranslation().getX(), RobotDimensions.BasePivotOffset.getTranslation().getY());
    m_extenderBaseLigament = m_liftRoot.append(new LoggedMechanismLigament2d("ExtenderBase", 0.89, 0, 6, m_extenderColor));
    m_extenderLigament = m_liftRoot.append(new LoggedMechanismLigament2d("Extender", 0.001, 0, 8, m_extenderColor));

    // Gripper
    m_gripperPivotLigament = m_extenderLigament.append(new LoggedMechanismLigament2d("GripperPivot", 0.410, -2.04, 1, m_gripperNeutralColor));
    m_gripperBodyLigament = m_gripperPivotLigament.append(new LoggedMechanismLigament2d("GripperBody", 0.2, 0, 5, m_gripperNeutralColor));
    m_gripperCoralLigament = m_gripperPivotLigament.append(new LoggedMechanismLigament2d("GripperCoral", 0.0001, 0, 10, m_gripperHasCoralColor));

    // Bucket
    m_bucketBaseLigament = m_liftRoot.append(new LoggedMechanismLigament2d("BucketBase", 0.423, 0, 1, m_bucketColor));
    m_bucketFlatLowerLigament = m_bucketBaseLigament.append(new LoggedMechanismLigament2d("BucketFlatLower", 0.114, -90, 2, m_bucketColor));
    m_bucketVerticalLowerLigament = m_bucketFlatLowerLigament.append(new LoggedMechanismLigament2d("BucketVerticalLower", 0.318, 90, 2, m_bucketPassableColor));
    m_bucketFlatUpperLigament = m_bucketVerticalLowerLigament.append(new LoggedMechanismLigament2d("BucketFlatUpper", 0.178, 90, 2, m_bucketPassableColor));
    // m_bucketVerticalUpperLigament = m_bucketVerticalLowerLigament.append(new LoggedMechanismLigament2d("BucketVerticalUpper", 0.099762, 0, 2, m_bucketColor));
    m_bucketSlotLigament = m_bucketFlatLowerLigament.append(new LoggedMechanismLigament2d("BucketSlot", 0.347, 60, 2, m_bucketColor));
  }

  @Override
  public void periodic() {
    IdLiftValues values = m_idLift.getLiftValues();

    // Set angles and length of IdLift parts
    m_extenderLigament.setLength(values.extenderLength == 0 ? 0.0001 : values.extenderLength);
    m_extenderLigament.setAngle(values.basePivotAngle);
    m_extenderBaseLigament.setAngle(values.basePivotAngle);
    m_bucketBaseLigament.setAngle(values.basePivotAngle);
    m_gripperBodyLigament.setAngle(values.gripperPivotAngle);
    m_gripperCoralLigament.setAngle(values.gripperPivotAngle);

    // Change color if holding a coral
    if (values.hasCoral) {
      m_gripperCoralLigament.setLength(0.301625);
    } else {
      m_gripperCoralLigament.setLength(0.001);
    }
  }
}
