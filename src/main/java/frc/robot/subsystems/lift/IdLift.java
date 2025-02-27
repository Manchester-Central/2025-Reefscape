// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import com.chaos131.gamepads.Gamepad;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.MidLiftConstants.ExtenderConstants;
import frc.robot.Constants.MidLiftConstants.LiftPoses;
import frc.robot.Robot;
import frc.robot.subsystems.shared.StateBasedSubsystem;
import frc.robot.subsystems.shared.SubsystemState;

import java.text.BreakIterator;

import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class IdLift extends StateBasedSubsystem<IdLift.LiftState> {
  /**
   * Possible values from the lift that can be used in lift parts and other areas
   * of the code
   * (without having to know about the base parts).
   */
  public class IdLiftValues {
    public Rotation2d basePivotAngle;
    public Rotation2d gripperPivotAngle;
    public double coralGripSpeed;
    public double extenderLength;
    public boolean isBasePivotAtSafeAngle;
    public boolean isExtenderAtSafeLength;
    public boolean hasCoral;
    public boolean isGripperPivotAtSafeAngle;
  }

  /**
   * Gets the latest lift values.
   */
  public IdLiftValues getLiftValues() {
    IdLiftValues values = new IdLiftValues();
    values.basePivotAngle = m_basePivot.getCurrentAngle();
    values.gripperPivotAngle = m_gripperPivot.getCurrentAngle();
    values.coralGripSpeed = m_gripper.getCoralGripSpeed();
    values.extenderLength = m_extender.getCurrentLength();
    values.isBasePivotAtSafeAngle = m_basePivot.isSafeAngle();
    values.isExtenderAtSafeLength = m_extender.isSafeLength();
    values.hasCoral = m_gripper.hasCoral();
    values.isGripperPivotAtSafeAngle = m_gripperPivot.isSafeAngle();
    return values;
  }

  public BasePivot m_basePivot = new BasePivot(this::getLiftValues); // TODO: UNDO public
  public Extender m_extender = new Extender(this::getLiftValues); // TODO: UNDO public
  public Gripper m_gripper = new Gripper(this::getLiftValues);
  public GripperPivot m_gripperPivot = new GripperPivot(this::getLiftValues);
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
    PREP_L1,
    PREP_L2,
    PREP_L3,
    PREP_L4, // Might need many prep states
    SCORE_L1,
    SCORE_L2,
    SCORE_L3,
    SCORE_L4,
    HOLD_CORAL,
    BOTTOM_BUCKET,
    TOP_BUCKET,
    PREP_CLIMB,
    POST_CLIMB;
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
    switch (getCurrentState()) {
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
      case PREP_L1:
        prepL1State();
        break;
      case PREP_L2:
        prepL2State();
        break;
      case PREP_L3:
        prepL3State();
        break;
      case PREP_L4:
        prepL4State();
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
      case HOLD_CORAL:
        holdCoralState();
        break;
      case BOTTOM_BUCKET:
        bottomBucketState();
        break;
      case TOP_BUCKET:
        topBucketState();
        break;
        case PREP_CLIMB:
        prepClimb();
        break;
        case POST_CLIMB:
        postClimb();
        break;
    }
  }

  private void startState() {
    if (ExtenderConstants.HasMagnetSensor && !m_extender.hasReachedMinimum()) {
      m_extender.setSpeed(-0.05); // Mr. Negative - Matt Bisson
      m_gripperPivot.setTargetAngle(LiftPoses.Stow.getGripperPivotAngle());
      return;
    }

    if (Robot.isSimulation()) {
      // changeState(LiftState.STOW);
      changeState(LiftState.STOW);
    } else {
      // changeState(LiftState.MANUAL);
      changeState(LiftState.STOW);
    }
  }

  private void manualState() {
    /*
     * m_basePivot.setSpeed(m_operator.getLeftY() * 0.131);
     * m_gripper.setCoralGripSpeed(0.0);
     * m_gripperPivot.setSpeed(0.0);
     * m_extender.setSpeed(m_operator.getRightY() * 0.5);
*/
    m_basePivot.setSpeed(m_operator.getRightY() * 0.131);

    m_extender.setSpeed(m_operator.getLeftY() * 0.5);

    if (m_operator.leftBumper().getAsBoolean()) {
      m_gripperPivot.setSpeed(0.2);
    } else if (m_operator.leftTrigger().getAsBoolean()) {
      m_gripperPivot.setSpeed(-0.2);
    } else {
      m_gripperPivot.setSpeed(0);
    }

    if (m_operator.rightBumper().getAsBoolean()) {
      m_gripper.setCoralGripSpeed(0.2);
    } else if (m_operator.rightTrigger().getAsBoolean()) {
      m_gripper.setCoralGripSpeed(-0.2);
    } else {
      m_gripper.setCoralGripSpeed(0);
    }

  }

  private void stowState() {
    if (m_gripper.hasCoral()) {
      changeState(m_extender.getCurrentLength() <= ExtenderConstants.BucketTopClearanceMeter 
      ? LiftState.BOTTOM_BUCKET 
      : LiftState.HOLD_CORAL);
      return;
    }
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
    if (m_gripper.hasCoral()) {
      changeState(LiftState.BOTTOM_BUCKET);
      m_gripper.setCoralGripSpeed(0.0);
      return;
    }
    m_basePivot.setTargetAngle(LiftPoses.HpIntake.getBasePivotAngle());
    m_extender.setTargetLength(LiftPoses.HpIntake.getExtensionMeters());
    m_gripperPivot.setTargetAngle(LiftPoses.HpIntake.getGripperPivotAngle());
    m_gripper.setCoralGripSpeed(-0.5);

    if (Robot.isSimulation() && getElapsedStateSeconds() > 2.0) {
      Gripper.hasCoralGrippedSim = true;
    }
  }

  private void prepL1State() {
    scoreHelper(LiftPoses.ScoreL1, true);
  }

  private void prepL2State() {
    scoreHelper(LiftPoses.ScoreL2, true);
  }

  private void prepL3State() {
    scoreHelper(LiftPoses.ScoreL3, true);
  }

  private void prepL4State() {
    scoreHelper(LiftPoses.ScoreL4, true);
  }

  private void scoreL1State() {
    scoreHelper(LiftPoses.ScoreL1, false);
  }

  private void scoreL2State() {
    scoreHelper(LiftPoses.ScoreL2, false);
  }

  private void scoreL3State() {
    scoreHelper(LiftPoses.ScoreL3, false);
  }

  private void scoreL4State() {
    scoreHelper(LiftPoses.ScoreL4, false);
  }

  private void holdCoralState() {
    m_basePivot.setTargetAngle(LiftPoses.HoldCoral.getBasePivotAngle());
    m_extender.setTargetLength(LiftPoses.HoldCoral.getExtensionMeters());
    m_gripperPivot.setTargetAngle(LiftPoses.HoldCoral.getGripperPivotAngle());
    if (!m_gripper.hasCoral()) {
      changeState(LiftState.TOP_BUCKET);
      return;
    }
  }

  private void bottomBucketState() {
    m_basePivot.setTargetAngle(LiftPoses.BottomBucket.getBasePivotAngle());
    m_extender.setTargetLength(LiftPoses.BottomBucket.getExtensionMeters());
    m_gripperPivot.setTargetAngle(LiftPoses.BottomBucket.getGripperPivotAngle());
    if (m_gripper.hasCoral() && m_gripperPivot.atTarget()) {
      changeState(LiftState.TOP_BUCKET);
      return;
    } else if (!m_gripper.hasCoral()) {
      changeState(LiftState.INTAKE_FROM_HP);
      return;
    }
  }

  private void topBucketState() {
    m_basePivot.setTargetAngle(LiftPoses.TopBucket.getBasePivotAngle());
    m_extender.setTargetLength(LiftPoses.TopBucket.getExtensionMeters());
    m_gripperPivot.setTargetAngle(LiftPoses.TopBucket.getGripperPivotAngle());
    if (m_gripper.hasCoral() && m_extender.atTarget()) {
      changeState(LiftState.HOLD_CORAL);
      return;
    } else if (!m_gripper.hasCoral() && m_gripperPivot.atTarget()) {
      changeState(LiftState.BOTTOM_BUCKET);
      return;
    }
  }

  private void scoreHelper(LiftPose liftPose, boolean isPrep) {
    if (!(m_gripper.hasCoral())) {
      changeState(LiftState.STOW);
      return;
    }
    m_basePivot.setTargetAngle(liftPose.getBasePivotAngle());
    m_extender.setTargetLength(liftPose.getExtensionMeters());
    m_gripperPivot.setTargetAngle(liftPose.getGripperPivotAngle());
    if (!isPrep && isPoseReady()) {
      m_gripper.setCoralGripSpeed(0.5);
    } else {
      m_gripper.setCoralGripSpeed(0);
    }

    if (!isPrep && Robot.isSimulation() && getElapsedStateSeconds() > 2.0) {
      Gripper.hasCoralGrippedSim = false;
    }
  }

  private void prepClimb() {
    m_basePivot.setTargetAngle(LiftPoses.ClimbPrep.getBasePivotAngle());
    m_gripperPivot.setTargetAngle(LiftPoses.ClimbPrep.getGripperPivotAngle());
    m_extender.setTargetLength(LiftPoses.ClimbPrep.getExtensionMeters());
  }

  private void postClimb() {
    m_basePivot.setTargetAngle(LiftPoses.Climb.getBasePivotAngle());
    m_gripperPivot.setTargetAngle(LiftPoses.Climb.getGripperPivotAngle());
    m_extender.setTargetLength(LiftPoses.Climb.getExtensionMeters());
  }

  /**
   * Set Motor to clean up. :3
   */
  public void setMotorCleanUp() {
    m_extender.setMotorCoast();
    // m_basePivot.setMotorCoast();
    m_gripperPivot.setMotorCoast();
  }

  /**
   * Set Motor to start up. :3
   */
  public void setMotorStartUp() {
    m_extender.setMotorBrake();
    // m_basePivot.setMotorBrake();
    m_gripperPivot.setMotorBrake();
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    super.periodic();
    Logger.recordOutput("CurrentState", getCurrentState().name());
  }
}
// RIP m_oldLift & m_oldGripper 2025-2025
