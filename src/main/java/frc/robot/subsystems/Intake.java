// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotDimensions;
import frc.robot.subsystems.shared.StateBasedSubsystem;
import frc.robot.subsystems.shared.SubsystemState;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

/**
 * The Intake subsystem for grabbing coral off the ground.
 */
public class Intake extends StateBasedSubsystem<Intake.IntakeState> {

  /**
   * The possible states of the Intake's state machine.
   */
  public enum IntakeState implements SubsystemState {
    START,
    STOW,
    DEPLOY,
    HANDOFF_PREP,
    HANDOFF;
  }

  // private double m_gearRatio = 10.0;
  // private double m_jkgMetersSquared = 1.0;
  private Rotation2d m_targetAngle = Rotation2d.fromDegrees(120);
  private double m_targetSpeed = 0.0;
  // private DCMotor m_pivotDcMotor = DCMotor.getKrakenX60(2);
  // private DCMotorSim m_pivotMotorSim =
  //     new DCMotorSim(
  //         LinearSystemId.createDCMotorSystem(m_pivotDcMotor, m_jkgMetersSquared, m_gearRatio),
  //         m_pivotDcMotor,
  //         0.001,
  //         0.001);
  // private ChaosTalonFx m_pivotMotor1 = new ChaosTalonFx(CanIdentifiers.IntakeMotor1CANID);
  // private ChaosTalonFx m_pivotMotor2 = new ChaosTalonFx(CanIdentifiers.IntakeMotorBCANID);
  // private PIDTuner m_pidTuner = new PIDTuner("IntakePivot", true, 0.1, 0.001, 0.0, this::tunePids);

  private IntakeSimulation m_physicSimIntake;
  private SwerveDriveSimulation m_simDriveTrain;
  private double m_simTimer;

  /** Creates a new Intake. */
  public Intake(SwerveDriveSimulation simdrivetrain) {
    super(IntakeState.START);
    // m_pivotMotor1.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // m_pivotMotor1.Configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // m_pivotMotor1.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    // m_pivotMotor1.Configuration.CurrentLimits.SupplyCurrentLimit = 40;
    // m_pivotMotor1.Configuration.Feedback.FeedbackSensorSource =
    //     FeedbackSensorSourceValue.RotorSensor;
    // m_pivotMotor1.Configuration.Feedback.SensorToMechanismRatio = 0.5; // TODO: get real value
    // m_pivotMotor1.Configuration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
    // m_pivotMotor1.applyConfig();

    // m_pivotMotor2.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // m_pivotMotor2.Configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // m_pivotMotor2.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    // m_pivotMotor2.Configuration.CurrentLimits.SupplyCurrentLimit = 40;
    // m_pivotMotor2.Configuration.Feedback.FeedbackSensorSource =
    //     FeedbackSensorSourceValue.RotorSensor;
    // m_pivotMotor2.Configuration.Feedback.SensorToMechanismRatio = 0.5; // TODO: get real value
    // m_pivotMotor2.Configuration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
    // m_pivotMotor2.applyConfig();

    // m_pivotMotor1.attachMotorSim(m_pivotMotorSim, m_gearRatio, true);
    // m_pivotMotor2.attachMotorSim(m_pivotMotorSim, m_gearRatio, false);

    m_simDriveTrain = simdrivetrain;
    m_physicSimIntake = IntakeSimulation.OverTheBumperIntake(
      // Specify the type of game pieces that the intake can collect
      "Coral",
      // Specify the drivetrain to which this intake is attached
      m_simDriveTrain,
      // Width of the intake
      Units.Meters.of(RobotDimensions.IntakeWidth),
      // The extension length of the intake beyond the robot's frame (when activated)
      Units.Meters.of(RobotDimensions.SimIntakeDistance),
      // The intake is mounted on the back side of the chassis
      IntakeSimulation.IntakeSide.FRONT,
      // The intake can hold up to 1 note
      1);
  }

  @Override
  protected void runStateMachine() {
    switch (getCurrentState()) {
      case START:
        changeState(IntakeState.STOW);
        break;

      case STOW:
        stowState();
        break;

      case DEPLOY:
        deployState();
        break;

      case HANDOFF_PREP:
        handoffPrepState();
        break;

      case HANDOFF:
        handoffState();
        break;

      default:
        break;
    }
  }

  private void stowState() {
    m_physicSimIntake.stopIntake();
    setTargetAngle(IntakeConstants.StowAngle);
    setTargetSpeed(IntakeConstants.StowSpeed);
  }

  private void deployState() {
    m_physicSimIntake.startIntake();
    setTargetAngle(IntakeConstants.DeployAngle);
    setTargetSpeed(IntakeConstants.DeploySpeed);
    if (hasGamePiece()) {
      changeState(IntakeState.HANDOFF_PREP);
      m_simTimer = Timer.getFPGATimestamp();
    }
  }

  private void handoffPrepState() {
    m_physicSimIntake.stopIntake();
    setTargetAngle(IntakeConstants.HandoffAngle);
    setTargetSpeed(IntakeConstants.HandoffPrepSpeed);
    var currTime = Timer.getFPGATimestamp();
    if (5 < (currTime - m_simTimer)) {
      changeState(IntakeState.HANDOFF);
    }
  }

  private void handoffState() {
    m_physicSimIntake.stopIntake();
    setTargetAngle(IntakeConstants.HandoffAngle);
    setTargetSpeed(IntakeConstants.HandoffSpeed);
    if (m_physicSimIntake.obtainGamePieceFromIntake()) {
      var piece = new ReefscapeCoralOnFly(
          m_simDriveTrain.getSimulatedDriveTrainPose().getTranslation(),
          new Translation2d(),
          m_simDriveTrain.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
          m_simDriveTrain.getSimulatedDriveTrainPose().getRotation(),
          Units.Meters.of(2.0),
          Units.MetersPerSecond.of(0),
          Units.Degrees.of(0));
      SimulatedArena.getInstance().addGamePieceProjectile(piece);
    }
  }

  /**
   * fml.
   */
  public void startPiecePickup() {
    changeState(IntakeState.DEPLOY);
  }

  // private void tunePids(PIDFValue pidfValue) {
  //   m_pivotMotor1.tunePid(pidfValue, 0.0);
  //   m_pivotMotor2.tunePid(pidfValue, 0.0);
  // }

  public void setTargetSpeed(double newSpeed) {
    m_targetSpeed = newSpeed;
  }

  public double getCurrentSpeed() {
    return m_targetSpeed;
  }

  /**
   * Sets the target angle and tries to drive there.
   */
  public void setTargetAngle(Rotation2d newAngle) {
    m_targetAngle = newAngle;
    // m_pivotMotor1.moveToPosition(newAngle.getDegrees());
    // m_pivotMotor2.moveToPosition(newAngle.getDegrees());
  }

  public Rotation2d getCurrentAngle() {
    return m_targetAngle; // TODO: change
    // return Rotation2d.fromDegrees(
    //     m_pivotMotor1.getPosition().getValueAsDouble()); // TODO get actual motor angle
  }

  @Override
  public void periodic() {
    // m_pidTuner.tune();
  }

  @Override
  public void simulationPeriodic() {
    // m_pivotMotor1.simUpdate();
    // m_pivotMotor2.simUpdate();
  }

  /**
   * Function that works for both real and simulated intakes.

   * @return true if the intake currently has a game piece in it
   */
  protected boolean hasGamePiece() {
    if (Robot.isSimulation()) {
      return m_physicSimIntake.getGamePiecesAmount() > 0;
    } else {
      return false;
    }
  }
}
