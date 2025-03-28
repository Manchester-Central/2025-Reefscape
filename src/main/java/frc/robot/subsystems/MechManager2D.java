// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotDimensions;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmValues;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;



/** A class for sending a 2d representation of our robot over network tables. */
public class MechManager2D extends SubsystemBase {
  private Arm m_arm;

  @AutoLogOutput(key = "Mech2d/Arm")
  private LoggedMechanism2d m_armBase;

  // Extender
  private LoggedMechanismRoot2d m_armRoot;
  private LoggedMechanismLigament2d m_extenderLigament;
  private LoggedMechanismLigament2d m_extenderBaseLigament;

  // Gripper
  private LoggedMechanismLigament2d m_gripperBaseLigament;
  private LoggedMechanismLigament2d m_gripperBaseVericalLigament;
  private LoggedMechanismLigament2d m_gripperWristLigament;
  private LoggedMechanismLigament2d m_gripperCenterLigament;
  private LoggedMechanismLigament2d m_gripperAlgaeSupportLigament;
  @SuppressWarnings("unused")
  private LoggedMechanismLigament2d m_gripperAlgaeWheelsLigament;
  private LoggedMechanismLigament2d m_gripperCoralWheelsLigament;
  private LoggedMechanismLigament2d m_gripperCoralFrontLigament;
  private LoggedMechanismLigament2d m_gripperCoralBackLigament;
  private LoggedMechanismLigament2d m_gripperAlgaeLigament;


  private final Color8Bit m_extenderColor = new Color8Bit(0, 0, 255);
  private final Color8Bit m_gripperNeutralColor = new Color8Bit(100, 100, 100);
  private final Color8Bit m_gripperForwardColor = new Color8Bit(0, 255, 0);
  private final Color8Bit m_gripperReverseColor = new Color8Bit(255, 0, 0);
  private final Color8Bit m_gripperHasCoralColor = new Color8Bit(255, 255, 255);
  private final Color8Bit m_gripperHasAlgaeColor = new Color8Bit(0, 128, 128);
  
  @AutoLogOutput(key = "Mech2d/Intake")
  private LoggedMechanism2d m_intakeBase;

  /**
   * Creates a new mech manager.
   */
  public MechManager2D(Arm arm) {
    m_arm = arm;

    m_armBase = new LoggedMechanism2d(2, 3);
    m_armRoot = m_armBase.getRoot("Arm", 0.8, 0.2);
    m_extenderBaseLigament = m_armRoot.append(new LoggedMechanismLigament2d("ExtenderBase", 0.89, 0, 6, m_extenderColor));
    m_extenderLigament = m_armRoot.append(new LoggedMechanismLigament2d("Extender", 0.001, 0, 8, m_extenderColor));

    // Gripper
    m_gripperBaseLigament = m_extenderLigament.append(new LoggedMechanismLigament2d("GripperBase", 0.1, -90, 4, m_gripperNeutralColor));
    m_gripperBaseVericalLigament = m_gripperBaseLigament.append(new LoggedMechanismLigament2d("GripperBaseVertical", 0.284828, 90, 2, m_gripperNeutralColor));
    m_gripperWristLigament = m_gripperBaseVericalLigament.append(new LoggedMechanismLigament2d("GripperWrist", 0.058165, 0, 2, m_gripperNeutralColor));
    m_gripperCenterLigament = m_gripperWristLigament.append(new LoggedMechanismLigament2d("GripperCenter", 0.232, 0, 2, m_gripperNeutralColor));
    m_gripperAlgaeSupportLigament = m_gripperCenterLigament.append(new LoggedMechanismLigament2d("GripperAlgaeSupport", 0.2, 64, 2, m_gripperNeutralColor));
    m_gripperAlgaeWheelsLigament = m_gripperAlgaeSupportLigament.append(new LoggedMechanismLigament2d("GripperAlgaeWheels", 0.05, 0, 2, m_gripperHasAlgaeColor));
    m_gripperCoralWheelsLigament = m_gripperCenterLigament.append(new LoggedMechanismLigament2d("CoralWheels", RobotDimensions.WristToCoralIntakeAxle, -90, 2, m_gripperNeutralColor));
    m_gripperCoralFrontLigament = m_gripperCoralWheelsLigament.append(new LoggedMechanismLigament2d("GripperCoralFront", 0.0001, 90, 10, m_gripperHasCoralColor));
    m_gripperCoralBackLigament = m_gripperCoralWheelsLigament.append(new LoggedMechanismLigament2d("GripperCoralBack", 0.0001, -90, 10, m_gripperHasCoralColor));
    m_gripperAlgaeLigament = m_gripperCenterLigament.append(new LoggedMechanismLigament2d("GripperAlgae", 0.0001, 0, 40, m_gripperHasAlgaeColor));
  }

  @Override
  public void periodic() {
    ArmValues values = m_arm.getArmValues();

    // Set angles and length of Arm parts
    m_extenderLigament.setLength(values.extenderLength == 0 ? 0.0001 : values.extenderLength);
    m_extenderLigament.setAngle(values.basePivotAngle.in(Degrees));
    m_extenderBaseLigament.setAngle(values.basePivotAngle.in(Degrees));
    m_gripperWristLigament.setAngle(values.gripperPivotAngle.in(Degrees));

    // Change coral gripper color
    if (values.coralGripSpeed == 0) {
      m_gripperCoralWheelsLigament.setColor(m_gripperNeutralColor);
    } else if (values.coralGripSpeed > 0) {
      m_gripperCoralWheelsLigament.setColor(m_gripperForwardColor);
    } else {
      m_gripperCoralWheelsLigament.setColor(m_gripperReverseColor);
    }

    // Change length if holding a coral
    if (values.hasCoral) {
      m_gripperCoralFrontLigament.setLength(RobotDimensions.WristToCoralFront.getX() - RobotDimensions.WristToCoralIntakeAxle);
      m_gripperCoralBackLigament.setLength(RobotDimensions.WristToCoralIntakeAxle - RobotDimensions.WristToCoralBack.getX());
    } else {
      m_gripperCoralFrontLigament.setLength(0.001);
      m_gripperCoralBackLigament.setLength(0.001);
    }

    // Change length if holding a coral
    if (values.hasAlgae) {
      m_gripperAlgaeLigament.setLength(0.4);
    } else {
      m_gripperAlgaeLigament.setLength(0.0001);
    }
  }
}
