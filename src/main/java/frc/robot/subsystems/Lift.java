// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {

  private MechanismLigament2d m_mechanism2dLift;

  // Simulator
  // TODO: add values to numMotors
  private DCMotor m_pivotGearBox = DCMotor.getVex775Pro(4);
  private SingleJointedArmSim m_pivotSim =
      new SingleJointedArmSim(
          m_pivotGearBox,
          10.0,
          4.0,
          2.0,
          Rotation2d.fromDegrees(30).getRadians(),
          Rotation2d.fromDegrees(75).getRadians(),
          true,
          Rotation2d.fromDegrees(35).getRadians());
  private DCMotor m_elevatorGearBox = DCMotor.getVex775Pro(4);

  private ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_elevatorGearBox, 10, 4, Units.inchesToMeters(2), 0, 1.5, false, 0, 0.005, 0);

  /** Creates a new Lift. */
  public Lift(MechanismRoot2d mechanismRoot) {
    m_mechanism2dLift =
        mechanismRoot.append(
            new MechanismLigament2d(
                "lift", m_elevatorSim.getPositionMeters(), 45, 6, new Color8Bit(0, 255, 0)));
  }

  @Override
  public void periodic() {
    m_elevatorSim.update(0.02);
  }

  public void setSpeed(double speed) {
    m_elevatorSim.setInput(speed);
    m_mechanism2dLift.setLength(m_elevatorSim.getPositionMeters());
  }
}
