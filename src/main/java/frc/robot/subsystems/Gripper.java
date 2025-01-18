// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
  private MechanismLigament2d m_mechanism2dGripper;

  // Simulator
  // TODO: add values to numMotors
  private DCMotor m_pivotGearBox = DCMotor.getKrakenX60(1);
  private SingleJointedArmSim m_pivotSim =
      new SingleJointedArmSim(
          m_pivotGearBox,
          24576 / 180.0,
          SingleJointedArmSim.estimateMOI(0.3, 5),
          0.3,
          Rotation2d.fromDegrees(-120).getRadians(),
          Rotation2d.fromDegrees(45).getRadians(),
          true,
          Rotation2d.fromDegrees(-20).getRadians());

  /** Creates a new Gripper. */
  public Gripper(MechanismLigament2d liftMech2d) {
    m_mechanism2dGripper =
        liftMech2d.append(
            new MechanismLigament2d("gripper", 0.3, 0, 4, new Color8Bit(255, 0, 255)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_pivotSim.update(0.02);
    m_mechanism2dGripper.setAngle(Rotation2d.fromRadians(m_pivotSim.getAngleRads()));
  }

  public void setPivotSpeed(double speed) {
    m_pivotSim.setInput(speed);
  }
}
