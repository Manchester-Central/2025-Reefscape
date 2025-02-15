// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import com.chaos131.gamepads.Gamepad;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.MidLiftConstants.LiftPoses;
import frc.robot.Robot;
import frc.robot.subsystems.shared.StateBasedSubsystem;
import frc.robot.subsystems.shared.SubsystemState;

/** Add your docs here. */
public class IdLift extends StateBasedSubsystem<IdLift.LiftState> {
  /**
   * Possible values from the lift that can be used in lift parts and other areas of the code 
   * (without having to know about the base parts).
   */
  public class IdLiftValues {
    public Rotation2d basePivotAngle;
    public Rotation2d gripperPivotAngle;
    public double coralGripSpeed;
    public double algaeGripSpeed;
    public double extenderLength;
    public boolean isBasePivotAtSafeAngle;
    public boolean isExtenderAtSafeLength;
    public boolean hasAlgaeGripped;
    public boolean hasCoralBackGripped;
    public boolean hasCoralFrontGripped;
  }

  /**
   * Gets the latest lift values.
   */
  public IdLiftValues getLiftValues() {
    IdLiftValues values = new IdLiftValues();
    values.basePivotAngle = m_basePivot.getCurrentAngle();
    values.gripperPivotAngle = m_gripperPivot.getCurrentAngle();
    values.coralGripSpeed = m_gripper.getCoralGripSpeed();
    values.algaeGripSpeed = m_gripper.getAlgaeGripSpeed();
    values.extenderLength = m_extender.getCurrentLength();
    values.isBasePivotAtSafeAngle = m_basePivot.isSafeAngle();
    values.isExtenderAtSafeLength = m_extender.isSafeLength();
    values.hasAlgaeGripped = m_gripper.hasAlgae();
    values.hasCoralBackGripped = m_gripper.hasCoralBack();
    values.hasCoralFrontGripped = m_gripper.hasCoralFront();
    return values;
  }

  private BasePivot m_basePivot = new BasePivot(this::getLiftValues);
  private Extender m_extender = new Extender(this::getLiftValues);
  private Gripper m_gripper = new Gripper(this::getLiftValues);
  private GripperPivot m_gripperPivot = new GripperPivot(this::getLiftValues);
  private Gamepad m_operator;

  /**
   * The possible states of the Lift's state machine.
   */
  public enum LiftState implements SubsystemState {
    MANUAL,
    START,
    STOW,
    INTAKE_FROM_FLOOR,
    INTAKE_FROM_HP, // Probably won't implement -Josh // nevermind -Josh
    SCORE_L1,
    SCORE_L2,
    SCORE_L3,
    SCORE_L4; // Might need many prep states
  }

  /** Creates a new Lift. */
  public IdLift(Gamepad operator) {
    super(LiftState.START);
    m_operator = operator;
  }

  private boolean isPoseReady() {
    return m_extender.atTarget() && m_basePivot.atTarget() && m_gripperPivot.atTarget();
  }

  @Override
  protected void runStateMachine() {
    switch (m_currentState) {
      case MANUAL:
        manualState();
        break;
      case START:
        startState();
        break;
      default:
      case STOW:
        stowState();
        break;
      case INTAKE_FROM_FLOOR:
        intakeFromFloorState();
        break;
      case INTAKE_FROM_HP:
        intakeFromHpState();
        break;
      case SCORE_L1:
        scoreL1State();
        break;
      case SCORE_L2:
        scoreL2State();
        break;
      case SCORE_L3:
        scoreL3State();
        break;
      case SCORE_L4:
        scoreL4State();
        break;
    }
  }

  private void startState() {
    // if (!m_extender.hasReachedMinimum()) {
    //   m_extender.setSpeed(-0.05); // Mr. Negative - Matt Bisson
    //   return;
    // }

    if (Robot.isSimulation()) {
      m_currentState = LiftState.STOW;
    } else {
      m_currentState = LiftState.MANUAL;
    }
  }

  private void manualState() {
    m_basePivot.setSpeed(0.0);
    m_gripper.setCoralGripSpeed(0.0);
    m_gripperPivot.setSpeed(0.0);
    m_extender.setSpeed(m_operator.getRightY() * 0.3);
  }

  private void stowState() {
    m_basePivot.setTargetAngle(LiftPoses.Stow.getBasePivotAngle());
    m_extender.setTargetLength(LiftPoses.Stow.getExtensionMeters());
    m_gripperPivot.setTargetAngle(LiftPoses.Stow.getGripperPivotAngle());
    m_gripper.setCoralGripSpeed(0.0);
  }

  private void intakeFromFloorState() {
    m_basePivot.setTargetAngle(LiftPoses.Handoff.getBasePivotAngle());
    m_extender.setTargetLength(LiftPoses.Handoff.getExtensionMeters());
    m_gripperPivot.setTargetAngle(LiftPoses.Handoff.getGripperPivotAngle());
    m_gripper.setCoralGripSpeed(0.5);
  }

  private void intakeFromHpState() {
    if (m_gripper.hasCoralBack() || m_gripper.hasCoralFront()) {
      m_currentState = LiftState.STOW;
      return;
    }
    m_basePivot.setTargetAngle(LiftPoses.HpIntake.getBasePivotAngle());
    m_extender.setTargetLength(LiftPoses.HpIntake.getExtensionMeters());
    m_gripperPivot.setTargetAngle(LiftPoses.HpIntake.getGripperPivotAngle());
    m_gripper.setCoralGripSpeed(-0.5);
  }

  private void scoreL1State() {
    scoreHelper(LiftPoses.ScoreL1, true);
  }

  private void scoreL2State() {
    scoreHelper(LiftPoses.ScoreL2, true);
  }

  private void scoreL3State() {
    scoreHelper(LiftPoses.ScoreL3, true);
  }

  private void scoreL4State() {
    scoreHelper(LiftPoses.ScoreL4, false);
  }

  private void scoreHelper(LiftPose liftPose, boolean isGripperReleaseForward) {
    if (!(m_gripper.hasCoralBack() || m_gripper.hasCoralFront())) {
      m_currentState = LiftState.STOW;
      return;
    }
    m_basePivot.setTargetAngle(liftPose.getBasePivotAngle());
    m_extender.setTargetLength(liftPose.getExtensionMeters());
    m_gripperPivot.setTargetAngle(liftPose.getGripperPivotAngle());
    if (isPoseReady()) {
      m_gripper.setCoralGripSpeed(isGripperReleaseForward ? 0.5 : -0.5);
    } else {
      m_gripper.setCoralGripSpeed(0);
    }
  }

  /**
   * Set Motor to clean up. :3
   */
  public void setMotorCleanup() {
    m_extender.setMotorCoast();
  }
}
// RIP m_oldLift & m_oldGripper 2025-2025
