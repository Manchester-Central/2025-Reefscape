// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.lift.IdLift;
import frc.robot.subsystems.lift.IdLift.IdLiftValues;

/** Add your docs here. */
public class Mech2DManager extends SubsystemBase {
  // @AutoLogOutput(key = "Manipulator")
  private Mechanism2d m_liftBase;
  private MechanismRoot2d m_liftRoot;
  private MechanismLigament2d m_extenderLigament;
  private MechanismLigament2d m_gripperLigament;
  private IdLift m_idLift;
  private Intake m_intake;
  private final Color8Bit kExtenderColor = new Color8Bit(0, 0, 255);
  private final Color8Bit kGripperNeutral = new Color8Bit(100, 100, 100);
  private final Color8Bit kGripperForward = new Color8Bit(0, 255, 0);
  private final Color8Bit kGripperReverse = new Color8Bit(255, 0, 0);

  public Mech2DManager(IdLift idLift, Intake intake) {
    m_idLift = idLift;
    m_intake = intake;
    m_liftBase = new Mechanism2d(2, 3);
    m_liftRoot = m_liftBase.getRoot("Lift", 1.4, 0.2);
    m_extenderLigament =
        m_liftRoot.append(new MechanismLigament2d("Extender", 0, 0, 0.4, kExtenderColor));
    m_gripperLigament =
        m_extenderLigament.append(new MechanismLigament2d("Gripper", 0.2, 0, 0.2, kGripperNeutral));
    SmartDashboard.putData("Mech2d/Lift", m_liftBase);
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
  }
}
