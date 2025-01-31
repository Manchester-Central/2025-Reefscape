// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.chaos131.pid.PIDFValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class ChaosTalonFx extends TalonFX {
  private final double m_gearRatio;
  private final DCMotorSim m_motorSimModel;
  private final boolean m_isMainSimMotor;
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0);
  public final TalonFXConfiguration Configuration = new TalonFXConfiguration();

  public ChaosTalonFx(int canID, double gearRatio, DCMotorSim dcMotorSim, boolean isMainSimMotor) {
    super(canID);
    this.m_gearRatio = gearRatio;
    m_motorSimModel = dcMotorSim;
    m_isMainSimMotor = isMainSimMotor;
  }

  /**
   * Tells the motor to handle updating the sim state. Copied/inspired from:
   * https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/simulation/simulation-intro.html
   */
  public void simUpdate() {
    var talonFXSim = getSimState();

    // set the supply voltage of the TalonFX
    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    if (m_isMainSimMotor) {
      // get the motor voltage of the TalonFX
      var motorVoltage = talonFXSim.getMotorVoltage();

      // use the motor voltage to calculate new position and velocity
      // using WPILib's DCMotorSim class for physics simulation
      m_motorSimModel.setInputVoltage(motorVoltage);
      m_motorSimModel.update(0.020); // assume 20 ms loop time
    }

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    talonFXSim.setRawRotorPosition(m_gearRatio * m_motorSimModel.getAngularPositionRotations());
    talonFXSim.setRotorVelocity(
        m_gearRatio * Units.radiansToRotations(m_motorSimModel.getAngularVelocityRadPerSec()));
  }

  public void applyConfig() {
    getConfigurator().apply(Configuration);
  }

  public void tunePID(PIDFValue pidValue, double kG) {
    var slot0 = new Slot0Configs();
    slot0.kP = pidValue.P;
    slot0.kI = pidValue.I;
    slot0.kD = pidValue.D;
    slot0.kG = kG;
    Configuration.Slot0 = slot0;
    applyConfig();
  }

  /** Tells the motor controller to move to the target position */
  public void moveToPosition(double position) {
    m_positionVoltage.Slot = 0;
    setControl(m_positionVoltage.withPosition(position));
  }
}
