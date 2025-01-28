package frc.robot.utils;

import com.chaos131.robot.ChaosRobot.Mode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class Motor {

  public Mode m_mode;
  public String m_name;
  public TalonFX m_talonfx;
  public CANcoder m_CANcoder;
  public DCMotor m_DCMotor;
  public ElevatorSim m_ElevatorSim;
  public final double GearRatio = 1.54;
  public final double ElevatorMassKg = 15.0;

  public TalonFXSimState m_simMotor;

  public Motor(Mode mode, String name, int motorNum, int CANNum) {
    mode = m_mode;
    name = m_name;
    m_talonfx = new TalonFX(motorNum);
    m_CANcoder = new CANcoder(CANNum);
    m_DCMotor = DCMotor.getKrakenX60(1);

    m_simMotor = m_talonfx.getSimState();
    m_ElevatorSim = new ElevatorSim(m_DCMotor, GearRatio, ElevatorMassKg, 0.076, 0, 2, false, 0);
  }

  public void periodic() {
    m_simMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
  }
}
