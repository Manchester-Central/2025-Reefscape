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
  private LoggedMechanismLigament2d m_gripperLigament;
  private LoggedMechanismLigament2d m_gripperFrontLigament;
  private LoggedMechanismLigament2d m_gripperBackLigament;
  private IdLift m_idLift;

  private final Color8Bit m_extenderColor = new Color8Bit(0, 0, 255);
  private final Color8Bit m_gripperNeutralColor = new Color8Bit(100, 100, 100);
  private final Color8Bit m_gripperForwardColor = new Color8Bit(0, 255, 0);
  private final Color8Bit m_gripperReverseColor = new Color8Bit(255, 0, 0);
  private final Color8Bit m_gripperHasCoralColor = new Color8Bit(255, 0, 255);

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
    m_extenderLigament = m_liftRoot.append(new LoggedMechanismLigament2d("Extender", 0, 0, 8, m_extenderColor));
    m_gripperLigament = m_extenderLigament.append(new LoggedMechanismLigament2d("Gripper", 0.2, 0, 8, m_gripperNeutralColor));
    m_gripperBackLigament = m_extenderLigament.append(new LoggedMechanismLigament2d("GripperBack", 0.09, 0, 10, m_gripperNeutralColor));
    m_gripperFrontLigament = m_gripperBackLigament.append(new LoggedMechanismLigament2d("GripperFront", 0.09, 0, 10, m_gripperNeutralColor));

    m_intakeBase = new LoggedMechanism2d(2, 3);
    m_intakeRoot = m_intakeBase.getRoot("Intake", 1.2, 0.2);
    m_innerIntakeLigament = m_intakeRoot.append(new LoggedMechanismLigament2d("InnerIntake", 0.3, 90, 8, m_innerIntakeColor));
    m_outerIntakeLigament = m_innerIntakeLigament.append(new LoggedMechanismLigament2d("OuterIntake", 0.2, -90, 10, m_intakeNeutralColor));
  }

  @Override
  public void periodic() {
    IdLiftValues values = m_idLift.getLiftValues();
    m_extenderLigament.setLength(values.extenderLength);
    m_extenderLigament.setAngle(values.basePivotAngle);
    m_gripperLigament.setAngle(values.gripperPivotAngle);
    m_gripperBackLigament.setAngle(values.gripperPivotAngle);
    if (values.gripperSpeed == 0) {
      m_gripperLigament.setColor(m_gripperNeutralColor);
    } else if (values.gripperSpeed > 0) {
      m_gripperLigament.setColor(m_gripperForwardColor);
    } else {
      m_gripperLigament.setColor(m_gripperReverseColor);
    }

    if (values.hasCoralBackGripped) {
      m_gripperBackLigament.setColor(m_gripperHasCoralColor);
    } else {
      m_gripperBackLigament.setColor(m_gripperNeutralColor);
    }

    if (values.hasCoralFrontGripped) {
      m_gripperFrontLigament.setColor(m_gripperHasCoralColor);
    } else {
      m_gripperFrontLigament.setColor(m_gripperNeutralColor);
    }

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
