// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.lift.IdLift;
import frc.robot.subsystems.lift.IdLift.IdLiftValues;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** A class for sending a 2d representation of our robot over network tables. */
public class MechManager2D extends SubsystemBase {
  @AutoLogOutput(key = "Mech2d/Lift")
  private LoggedMechanism2d m_liftBase;

  private LoggedMechanismRoot2d m_liftRoot;
  private LoggedMechanismLigament2d m_extenderLigament;
  private LoggedMechanismLigament2d m_extenderBaseLigament;
  private LoggedMechanismLigament2d m_gripperBaseLigament;
  private LoggedMechanismLigament2d m_gripperCenterLigament;
  private LoggedMechanismLigament2d m_gripperBottomLigament;
  private LoggedMechanismLigament2d m_gripperBottomMotorBaseLigament;
  private LoggedMechanismLigament2d m_gripperBottomMotorIndicatorFrontLigament;
  private LoggedMechanismLigament2d m_gripperBottomMotorIndicatorBackLigament;
  private LoggedMechanismLigament2d m_gripperFrontLigament;
  private LoggedMechanismLigament2d m_gripperBackLigament;
  private LoggedMechanismLigament2d m_gripperAlgaeHolderLigament;
  private LoggedMechanismLigament2d m_gripperAlgaePreviewLigament;
  private IdLift m_idLift;

  private final Color8Bit m_extenderColor = new Color8Bit(0, 0, 255);
  private final Color8Bit m_gripperNeutralColor = new Color8Bit(100, 100, 100);
  private final Color8Bit m_gripperForwardColor = new Color8Bit(0, 255, 0);
  private final Color8Bit m_gripperReverseColor = new Color8Bit(255, 0, 0);
  private final Color8Bit m_gripperHasCoralColor = new Color8Bit(255, 255, 255);
  private final Color8Bit m_gripperHasAlgaeColor = new Color8Bit(0, 128, 128);

  @AutoLogOutput(key = "Mech2d/Intake")
  private LoggedMechanism2d m_intakeBase;

  private LoggedMechanismRoot2d m_intakeRoot;
  private LoggedMechanismLigament2d m_innerIntakeLigament;
  private LoggedMechanismLigament2d m_outerIntakeLigament;
  private Intake m_intake;

  private final Color8Bit m_innerIntakeColor = new Color8Bit(255, 0, 255);
  private final Color8Bit m_intakeNeutralColor = new Color8Bit(100, 100, 100);
  private final Color8Bit m_intakeForwardColor = new Color8Bit(0, 255, 0);
  private final Color8Bit m_intakeReverseColor = new Color8Bit(255, 0, 0);

  /**
   * Creates a new mech manager.
   */
  public MechManager2D(IdLift idLift, Intake intake) {
    m_idLift = idLift;
    m_intake = intake;

    m_liftBase = new LoggedMechanism2d(2, 3);
    m_liftRoot = m_liftBase.getRoot("Lift", 0.8, 0.2);
    m_extenderBaseLigament = m_liftRoot.append(new LoggedMechanismLigament2d("ExtenderBase", 0.89, 0, 6, m_extenderColor));
    m_extenderLigament = m_liftRoot.append(new LoggedMechanismLigament2d("Extender", 0.001, 0, 8, m_extenderColor));

    // Gripper
    m_gripperBaseLigament = m_extenderLigament.append(new LoggedMechanismLigament2d("GripperBase", 0.31, -51, 4, m_gripperNeutralColor));
    m_gripperCenterLigament = m_gripperBaseLigament.append(new LoggedMechanismLigament2d("GripperCenter", 0.005, 0, 2, m_gripperNeutralColor));
    m_gripperBottomLigament = m_gripperCenterLigament.append(new LoggedMechanismLigament2d("GripperBottom", 0.1, -90, 2, m_gripperNeutralColor));
    m_gripperBottomMotorBaseLigament = m_gripperBottomLigament.append(new LoggedMechanismLigament2d("GripperBottomMotorBase", 0.02, 0, 2, m_gripperNeutralColor));
    m_gripperBottomMotorIndicatorFrontLigament = m_gripperBottomMotorBaseLigament.append(new LoggedMechanismLigament2d("GripperBottomIndicatorFront", 0.1, 90, 4, m_gripperNeutralColor));
    m_gripperBottomMotorIndicatorBackLigament = m_gripperBottomMotorBaseLigament.append(new LoggedMechanismLigament2d("GripperBottomIndicatorBack", 0.1, -90, 4, m_gripperNeutralColor));
    m_gripperBackLigament = m_gripperBottomLigament.append(new LoggedMechanismLigament2d("GripperBack", 0.0, -90, 6, m_gripperHasCoralColor));
    m_gripperFrontLigament = m_gripperBottomLigament.append(new LoggedMechanismLigament2d("GripperFront", 0.0, 90, 6, m_gripperHasCoralColor));
    m_gripperAlgaeHolderLigament = m_gripperCenterLigament.append(new LoggedMechanismLigament2d("GripperAlgaeHolder", 0.42, 90, 2, m_gripperNeutralColor));
    m_gripperAlgaePreviewLigament = m_gripperCenterLigament.append(new LoggedMechanismLigament2d("GripperAlgaePreview", 0.0, 90, 10, m_gripperHasAlgaeColor));

    m_intakeBase = new LoggedMechanism2d(2, 3);
    m_intakeRoot = m_intakeBase.getRoot("Intake", 1.2, 0.2);
    m_innerIntakeLigament = m_intakeRoot.append(new LoggedMechanismLigament2d("InnerIntake", 0.3, 90, 8, m_innerIntakeColor));
    m_outerIntakeLigament = m_innerIntakeLigament.append(new LoggedMechanismLigament2d("OuterIntake", 0.2, -90, 10, m_intakeNeutralColor));
  }

  @Override
  public void periodic() {
    IdLiftValues values = m_idLift.getLiftValues();

    // Set angles and length of IdLift parts
    m_extenderLigament.setLength(values.extenderLength);
    m_extenderLigament.setAngle(values.basePivotAngle);
    m_extenderBaseLigament.setAngle(values.basePivotAngle);
    m_gripperCenterLigament.setAngle(values.gripperPivotAngle);

    // Change coral gripper color
    if (values.coralGripSpeed == 0) {
      m_gripperBottomMotorIndicatorFrontLigament.setColor(m_gripperNeutralColor);
      m_gripperBottomMotorIndicatorBackLigament.setColor(m_gripperNeutralColor);
    } else if (values.coralGripSpeed > 0) {
      m_gripperBottomMotorIndicatorFrontLigament.setColor(m_gripperForwardColor);
      m_gripperBottomMotorIndicatorBackLigament.setColor(m_gripperForwardColor);
    } else {
      m_gripperBottomMotorIndicatorFrontLigament.setColor(m_gripperReverseColor);
      m_gripperBottomMotorIndicatorBackLigament.setColor(m_gripperReverseColor);
    }
    
    // Change algae gripper color
    if (values.algaeGripSpeed == 0) {
      m_gripperAlgaeHolderLigament.setColor(m_gripperNeutralColor);
    } else if (values.algaeGripSpeed > 0) {
      m_gripperAlgaeHolderLigament.setColor(m_gripperForwardColor);
    } else {
      m_gripperAlgaeHolderLigament.setColor(m_gripperReverseColor);
    }

    // Show algae preview if holding an algae
    if (values.hasAlgaeGripped) {
      m_gripperAlgaePreviewLigament.setLength(0.3);
    } else {
      m_gripperAlgaePreviewLigament.setLength(0.001);
    }

    // Change color if holding a coral in front
    if (values.hasCoralBackGripped) {
      m_gripperBackLigament.setLength(0.09);
    } else {
      m_gripperBackLigament.setLength(0.001);
    }

    // Change color if holding a coral in back
    if (values.hasCoralFrontGripped) {
      m_gripperFrontLigament.setLength(0.09);
    } else {
      m_gripperFrontLigament.setLength(0.001);
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
