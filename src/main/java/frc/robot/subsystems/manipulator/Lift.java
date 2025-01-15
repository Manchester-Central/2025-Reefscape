// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {

  private MechanismLigament2d m_mechanism2dLift; 

  //Simulator
  private DCMotor m_liftGearBox = DCMotor.getVex775Pro(4);
  private ElevatorSim m_sim = new ElevatorSim(m_liftGearBox, 10, 4, Units.inchesToMeters(2), 0, 1.5, false, 0, 0.005, 0);

  /** Creates a new Lift. */
  public Lift(MechanismRoot2d mechanismRoot) {
    m_mechanism2dLift = mechanismRoot.append(new MechanismLigament2d("lift", m_sim.getPositionMeters(), 45, 6, new Color8Bit(0, 255, 0)));

  }

  @Override
  public void periodic() {
    m_sim.update(0.02);
  }

  public void setSpeed(double speed){
    m_sim.setInput(speed);
    m_mechanism2dLift.setLength(m_sim.getPositionMeters());
  }
}


