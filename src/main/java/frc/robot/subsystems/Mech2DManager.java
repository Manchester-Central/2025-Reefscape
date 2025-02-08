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

/** Add your docs here. */
public class Mech2DManager extends SubsystemBase {
  @AutoLogOutput(key = "Mech2d/Lift")
  private LoggedMechanism2d m_liftBase;

  private LoggedMechanismRoot2d m_liftRoot;
  private LoggedMechanismLigament2d m_extenderLigament;
  private LoggedMechanismLigament2d m_gripperLigament;
  private IdLift m_idLift;

  private final Color8Bit kExtenderColor = new Color8Bit(0, 0, 255);
  private final Color8Bit kGripperNeutral = new Color8Bit(100, 100, 100);
  private final Color8Bit kGripperForward = new Color8Bit(0, 255, 0);
  private final Color8Bit kGripperReverse = new Color8Bit(255, 0, 0);

  @AutoLogOutput(key = "Mech2d/Intake")
  private LoggedMechanism2d m_intakeBase;

  private LoggedMechanismRoot2d m_intakeRoot;
  private LoggedMechanismLigament2d m_innerIntakeLigament;
  private LoggedMechanismLigament2d m_outerIntakeLigament;
  private Intake m_intake;

  private final Color8Bit kInnerIntakeColor = new Color8Bit(255, 0, 255);
  private final Color8Bit kIntakeNeutralColor = new Color8Bit(100, 100, 100);
  private final Color8Bit kIntakeForwardColor = new Color8Bit(0, 255, 0);
  private final Color8Bit kIntakeReverseColor = new Color8Bit(255, 0, 0);

  public Mech2DManager(IdLift idLift, Intake intake) {
    m_idLift = idLift;
    m_intake = intake;

    m_liftBase = new LoggedMechanism2d(2, 3);
    m_liftRoot = m_liftBase.getRoot("Lift", 0.8, 0.2);
    m_extenderLigament =
        m_liftRoot.append(new LoggedMechanismLigament2d("Extender", 0, 0, 8, kExtenderColor));
    m_gripperLigament =
        m_extenderLigament.append(
            new LoggedMechanismLigament2d("Gripper", 0.2, 0, 10, kGripperNeutral));
    // SmartDashboard.putData("Mech2d/Lift", m_liftBase);

    m_intakeBase = new LoggedMechanism2d(2, 3);
    m_intakeRoot = m_intakeBase.getRoot("Intake", 1.2, 0.2);
    m_innerIntakeLigament =
        m_intakeRoot.append(
            new LoggedMechanismLigament2d("InnerIntake", 0.3, 90, 8, kInnerIntakeColor));
    m_outerIntakeLigament =
        m_innerIntakeLigament.append(
            new LoggedMechanismLigament2d("OuterIntake", 0.2, -90, 10, kIntakeNeutralColor));
    // Logger.recordOutput("Mech2d/Intake", m_intakeBase);
  }

  @Override
  public void periodic() {
    IdLiftValues values = m_idLift.getLiftValues();
    m_extenderLigament.setLength(values.extenderLength);
    m_extenderLigament.setAngle(values.basePivotAngle);
    m_gripperLigament.setAngle(values.gripperPivotAngle);
    if (values.gripperSpeed == 0) {
      m_gripperLigament.setColor(kGripperNeutral);
    } else if (values.gripperSpeed > 0) {
      m_gripperLigament.setColor(kGripperForward);
    } else {
      m_gripperLigament.setColor(kGripperReverse);
    }

    m_innerIntakeLigament.setAngle(m_intake.getCurrentAngle());
    if (m_intake.getCurrentSpeed() > 0) {
      m_outerIntakeLigament.setColor(kIntakeForwardColor);
    } else if (m_intake.getCurrentSpeed() < 0) {
      m_outerIntakeLigament.setColor(kIntakeReverseColor);
    } else {
      m_outerIntakeLigament.setColor(kIntakeNeutralColor);
    }
  }
}
