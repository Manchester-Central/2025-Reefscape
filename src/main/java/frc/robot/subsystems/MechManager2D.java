// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmValues;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** A class for sending a 2d representation of our robot over network tables. */
public class MechManager2D extends SubsystemBase {
  private Arm m_arm;
  private Intake m_intake;

  @AutoLogOutput(key = "Mech2d/Arm")
  private LoggedMechanism2d m_armBase;

  // Extender
  private LoggedMechanismRoot2d m_armRoot;
  private LoggedMechanismLigament2d m_extenderLigament;
  private LoggedMechanismLigament2d m_extenderBaseLigament;

  // Gripper
  private LoggedMechanismLigament2d m_gripperBaseLigament;
  private LoggedMechanismLigament2d m_gripperBaseVericalLigament;
  private LoggedMechanismLigament2d m_gripperPivotLigament;
  private LoggedMechanismLigament2d m_gripperWheelsLigament;
  private LoggedMechanismLigament2d m_gripperCoralLigament;

  // Bucket
  private LoggedMechanismLigament2d m_bucketBaseLigament;
  private LoggedMechanismLigament2d m_bucketFlatLowerLigament;
  private LoggedMechanismLigament2d m_bucketFlatUpperLigament;
  private LoggedMechanismLigament2d m_bucketVerticalLowerLigament;
  private LoggedMechanismLigament2d m_bucketVerticalUpperLigament;
  private LoggedMechanismLigament2d m_bucketSlotLigament;

  private final Color8Bit m_extenderColor = new Color8Bit(0, 0, 255);
  private final Color8Bit m_gripperNeutralColor = new Color8Bit(100, 100, 100);
  private final Color8Bit m_gripperForwardColor = new Color8Bit(0, 255, 0);
  private final Color8Bit m_gripperReverseColor = new Color8Bit(255, 0, 0);
  private final Color8Bit m_gripperHasCoralColor = new Color8Bit(255, 255, 255);
  private final Color8Bit m_gripperHasAlgaeColor = new Color8Bit(0, 128, 128);
  private final Color8Bit m_bucketColor = new Color8Bit(150, 150, 150);
  private final Color8Bit m_bucketPassableColor = new Color8Bit(200, 200, 200);

  @AutoLogOutput(key = "Mech2d/Intake")
  private LoggedMechanism2d m_intakeBase;

  private LoggedMechanismRoot2d m_intakeRoot;
  private LoggedMechanismLigament2d m_innerIntakeLigament;
  private LoggedMechanismLigament2d m_outerIntakeLigament;

  private final Color8Bit m_innerIntakeColor = new Color8Bit(255, 0, 255);
  private final Color8Bit m_intakeNeutralColor = new Color8Bit(100, 100, 100);
  private final Color8Bit m_intakeForwardColor = new Color8Bit(0, 255, 0);
  private final Color8Bit m_intakeReverseColor = new Color8Bit(255, 0, 0);

  /**
   * Creates a new mech manager.
   */
  public MechManager2D(Arm arm, Intake intake) {
    m_arm = arm;
    m_intake = intake;

    m_armBase = new LoggedMechanism2d(2, 3);
    m_armRoot = m_armBase.getRoot("Arm", 0.8, 0.2);
    m_extenderBaseLigament = m_armRoot.append(new LoggedMechanismLigament2d("ExtenderBase", 0.89, 0, 6, m_extenderColor));
    m_extenderLigament = m_armRoot.append(new LoggedMechanismLigament2d("Extender", 0.001, 0, 8, m_extenderColor));

    // Gripper
    m_gripperBaseLigament = m_extenderLigament.append(new LoggedMechanismLigament2d("GripperBase", 0.1, -90, 4, m_gripperNeutralColor));
    m_gripperBaseVericalLigament = m_gripperBaseLigament.append(new LoggedMechanismLigament2d("GripperBaseVertical", 0.284828, 90, 2, m_gripperNeutralColor));
    m_gripperPivotLigament = m_gripperBaseVericalLigament.append(new LoggedMechanismLigament2d("GripperPivot", 0.058165, 0, 2, m_gripperNeutralColor));
    m_gripperWheelsLigament = m_gripperPivotLigament.append(new LoggedMechanismLigament2d("GripperWheels", 0.058165, 0, 5, m_gripperNeutralColor));
    m_gripperCoralLigament = m_gripperPivotLigament.append(new LoggedMechanismLigament2d("GripperCoral", 0.0001, 0, 10, m_gripperHasCoralColor));

    // Bucket
    m_bucketBaseLigament = m_armRoot.append(new LoggedMechanismLigament2d("BucketBase", 0.40, 0, 2, m_bucketColor));
    m_bucketFlatLowerLigament = m_bucketBaseLigament.append(new LoggedMechanismLigament2d("BucketFlatLower", 0.186307, -90, 2, m_bucketColor));
    m_bucketVerticalLowerLigament = m_bucketFlatLowerLigament.append(new LoggedMechanismLigament2d("BucketVerticalLower", 0.313388, 90, 2, m_bucketPassableColor));
    m_bucketFlatUpperLigament = m_bucketVerticalLowerLigament.append(new LoggedMechanismLigament2d("BucketFlatUpper", 0.261715, 90, 2, m_bucketPassableColor));
    m_bucketVerticalUpperLigament = m_bucketVerticalLowerLigament.append(new LoggedMechanismLigament2d("BucketVerticalUpper", 0.099762, 0, 2, m_bucketColor));
    m_bucketSlotLigament = m_bucketFlatLowerLigament.append(new LoggedMechanismLigament2d("BucketSlot", 0.364320, 60, 2, m_bucketColor));

    // Intake
    m_intakeBase = new LoggedMechanism2d(2, 3);
    m_intakeRoot = m_intakeBase.getRoot("Intake", 1.2, 0.2);
    m_innerIntakeLigament = m_intakeRoot.append(new LoggedMechanismLigament2d("InnerIntake", 0.3, 90, 8, m_innerIntakeColor));
    m_outerIntakeLigament = m_innerIntakeLigament.append(new LoggedMechanismLigament2d("OuterIntake", 0.2, -90, 10, m_intakeNeutralColor));
  }

  @Override
  public void periodic() {
    ArmValues values = m_arm.ArmValues();

    // Set angles and length of Arm parts
    m_extenderLigament.setLength(values.extenderLength == 0 ? 0.0001 : values.extenderLength);
    m_extenderLigament.setAngle(values.basePivotAngle);
    m_extenderBaseLigament.setAngle(values.basePivotAngle);
    m_bucketBaseLigament.setAngle(values.basePivotAngle);
    m_gripperPivotLigament.setAngle(values.gripperPivotAngle);

    // Change coral gripper color
    if (values.coralGripSpeed == 0) {
      m_gripperWheelsLigament.setColor(m_gripperNeutralColor);
    } else if (values.coralGripSpeed > 0) {
      m_gripperWheelsLigament.setColor(m_gripperForwardColor);
    } else {
      m_gripperWheelsLigament.setColor(m_gripperReverseColor);
    }

    // Change color if holding a coral
    if (values.hasCoral) {
      m_gripperCoralLigament.setLength(0.301625);
    } else {
      m_gripperCoralLigament.setLength(0.001);
    }

    // Control angle and color of intake
    m_innerIntakeLigament.setAngle(m_intake.getCurrentAngle());
    if (m_intake.getCurrentSpeed() > 0) {
      m_outerIntakeLigament.setColor(m_intakeForwardColor);
    } else if (m_intake.getCurrentSpeed() < 0) {
      m_outerIntakeLigament.setColor(m_intakeReverseColor);
    } else {
      m_outerIntakeLigament.setColor(m_intakeNeutralColor);
    }
  }
}
