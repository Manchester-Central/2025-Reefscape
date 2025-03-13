// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.ArmConstants.ExtenderConstants;
import frc.robot.Constants.FieldDimensions;
import frc.robot.Constants.ArmConstants.ArmPoses;
import frc.robot.Robot;
import frc.robot.subsystems.shared.StateBasedSubsystem;
import frc.robot.subsystems.shared.SubsystemState;
import frc.robot.utils.IkEquations;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Arm extends StateBasedSubsystem<Arm.ArmState> {
  /**
   * Possible values from the arm that can be used in arm parts and other areas
   * of the code
   * (without having to know about the base parts).
   */
  public class ArmValues {
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
   * Gets the latest arm values.
   */
  public ArmValues getArmValues() {
    ArmValues values = new ArmValues();
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

  public BasePivot m_basePivot = new BasePivot(this::getArmValues); // TODO: UNDO public
  public Extender m_extender = new Extender(this::getArmValues); // TODO: UNDO public
  public Gripper m_gripper = new Gripper(this::getArmValues);
  public GripperPivot m_gripperPivot = new GripperPivot(this::getArmValues);
  private Gamepad m_operator;
  private Pose3d m_ikTargetPose = new Pose3d(0, 0, FieldDimensions.Reef3Meters, new Rotation3d());
  private Supplier<Pose2d> m_robotPoseSupplier;

  /**
   * The possible states of the Arm's state machine.
   */
  public enum ArmState implements SubsystemState {
    MANUAL,
    START,
    STOW,
    IKSOLVER,
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
    ALGAE_HIGH,
    ALGAE_LOW,
    HOLD_CORAL,
    BOTTOM_BUCKET,
    TOP_BUCKET,
    PREP_CLIMB,
    POST_CLIMB;
  }

  /** Creates a new Arm. */
  public Arm(Gamepad operator, Supplier<Pose2d> robotPose) {
    super(ArmState.START);
    m_operator = operator;
    m_robotPoseSupplier = robotPose;
  }

  private boolean isPoseReady() {
    return m_extender.atTarget() && m_basePivot.atTarget() && m_gripperPivot.atTarget();
  }

  private boolean isPoseClose() {
    return m_extender.atClose() && m_basePivot.atClose() && m_gripperPivot.atClose();
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
      case IKSOLVER:
        ikSolverState();
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
      case ALGAE_HIGH:
        algaeHighState();
        break;
      case ALGAE_LOW:
        algaeLowState();
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
      m_gripperPivot.setTargetAngle(ArmPoses.Stow.getGripperPivotAngle());
      return;
    }

    if (Robot.isSimulation()) {
      // changeState(ArmState.STOW);
      changeState(ArmState.IKSOLVER);
    } else {
      // changeState(ArmState.MANUAL);
      changeState(ArmState.STOW);
    }
  }


  public void setIkSolverTarget(Pose3d ikTarget) {
    m_ikTargetPose = ikTarget;
  }

  /**
   * See The drawio diagram 'IKMath' in documents for details on the math.
   */
  private void ikSolverState() {
    // Calculate IK root
    Pose3d robotPose = new Pose3d(m_robotPoseSupplier.get()); // This becomes 0,0 very soon
    
    ArmPose mechanismPose = IkEquations.getPivotLiftPivot(robotPose, m_ikTargetPose);

    // Logging
    Logger.recordOutput("IkSolver/targetEndEffector", m_ikTargetPose);
    Logger.recordOutput("IkSolver/Extension", mechanismPose.getExtensionMeters());
    Logger.recordOutput("IkSolver/BasePivot", mechanismPose.getBasePivotAngle());
    Logger.recordOutput("IkSolver/GripperPivot", mechanismPose.getGripperPivotAngle());

    // Now apply them!
    m_extender.setTargetLength(mechanismPose.getExtensionMeters());
    m_basePivot.setTargetAngle(mechanismPose.getBasePivotAngle());
    m_gripperPivot.setTargetAngle(mechanismPose.getGripperPivotAngle());
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
      ? ArmState.BOTTOM_BUCKET 
      : ArmState.HOLD_CORAL);
      return;
    }
    m_basePivot.setTargetAngle(ArmPoses.Stow.getBasePivotAngle());
    m_extender.setTargetLength(ArmPoses.Stow.getExtensionMeters());
    m_gripperPivot.setTargetAngle(ArmPoses.Stow.getGripperPivotAngle());
    m_gripper.setCoralGripSpeed(0.0);
  }

  private void intakeFromFloorState() {
    m_basePivot.setTargetAngle(ArmPoses.Handoff.getBasePivotAngle());
    m_extender.setTargetLength(ArmPoses.Handoff.getExtensionMeters());
    m_gripperPivot.setTargetAngle(ArmPoses.Handoff.getGripperPivotAngle());
    m_gripper.setCoralGripSpeed(0.5);
  }

  private void intakeFromHpState() {
    if (m_gripper.hasCoral()) {
      changeState(ArmState.BOTTOM_BUCKET);
      m_gripper.setCoralGripSpeed(0.0);
      return;
    }
    m_basePivot.setTargetAngle(ArmPoses.HpIntake.getBasePivotAngle());
    m_extender.setTargetLength(ArmPoses.HpIntake.getExtensionMeters());
    m_gripperPivot.setTargetAngle(ArmPoses.HpIntake.getGripperPivotAngle());
    m_gripper.setCoralGripSpeed(-0.5);

    if (Robot.isSimulation() && getElapsedStateSeconds() > 2.0) {
      Gripper.hasCoralGrippedSim = true;
    }
  }

  private void prepL1State() {
    scoreHelper(ArmPoses.ScoreL1, true);
  }

  private void prepL2State() {
    scoreHelper(ArmPoses.ScoreL2, true);
  }

  private void prepL3State() {
    scoreHelper(ArmPoses.ScoreL3, true);
  }

  private void prepL4State() {
    scoreHelper(ArmPoses.ScoreL4, true);
  }

  private void scoreL1State() {
    scoreHelper(ArmPoses.ScoreL1, false);
  }

  private void scoreL2State() {
    scoreHelper(ArmPoses.ScoreL2, false);
  }

  private void scoreL3State() {
    scoreHelper(ArmPoses.ScoreL3, false);
  }

  private void scoreL4State() {
    scoreHelper(ArmPoses.ScoreL4, false);
  }

  private void algaeHighState() {
    if (!getArmValues().hasCoral) {
      m_basePivot.setTargetAngle(ArmPoses.AlgaeHigh.getBasePivotAngle());
      m_extender.setTargetLength(ArmPoses.AlgaeHigh.getExtensionMeters());
      m_gripperPivot.setTargetAngle(ArmPoses.AlgaeHigh.getGripperPivotAngle());
      m_gripper.setCoralGripSpeed(-0.5);
    } else {
      changeState(ArmState.HOLD_CORAL);
    }
  }

  private void algaeLowState() {
    if (!getArmValues().hasCoral) {
      m_basePivot.setTargetAngle(ArmPoses.AlgaeLow.getBasePivotAngle());
      m_extender.setTargetLength(ArmPoses.AlgaeLow.getExtensionMeters());
      m_gripperPivot.setTargetAngle(ArmPoses.AlgaeLow.getGripperPivotAngle());
      m_gripper.setCoralGripSpeed(-0.5);
    } else {
      changeState(ArmState.HOLD_CORAL);
    }
  }

  private void holdCoralState() {
    m_basePivot.setTargetAngle(ArmPoses.HoldCoral.getBasePivotAngle());
    m_extender.setTargetLength(ArmPoses.HoldCoral.getExtensionMeters());
    m_gripperPivot.setTargetAngle(ArmPoses.HoldCoral.getGripperPivotAngle());
    if (!m_gripper.hasCoral()) {
      changeState(ArmState.TOP_BUCKET);
      return;
    }
  }

  private void bottomBucketState() {
    m_basePivot.setTargetAngle(ArmPoses.BottomBucket.getBasePivotAngle());
    m_extender.setTargetLength(ArmPoses.BottomBucket.getExtensionMeters());
    m_gripperPivot.setTargetAngle(ArmPoses.BottomBucket.getGripperPivotAngle());
    if (m_gripper.hasCoral() && m_gripperPivot.atTarget()) {
      changeState(ArmState.TOP_BUCKET);
      return;
    } else if (!m_gripper.hasCoral()) {
      changeState(ArmState.STOW);
      return;
    }
  }

  private void topBucketState() {
    m_basePivot.setTargetAngle(ArmPoses.TopBucket.getBasePivotAngle());
    m_extender.setTargetLength(ArmPoses.TopBucket.getExtensionMeters());
    m_gripperPivot.setTargetAngle(ArmPoses.TopBucket.getGripperPivotAngle());
    if (m_gripper.hasCoral() && m_extender.atTarget()) {
      changeState(ArmState.HOLD_CORAL);
      return;
    } else if (!m_gripper.hasCoral() && m_gripperPivot.atTarget()) {
      changeState(ArmState.BOTTOM_BUCKET);
      return;
    }
  }

  private void scoreHelper(ArmPose ArmPose, boolean isPrep) {
    if (!(m_gripper.hasCoral())) {
      changeState(ArmState.STOW);
      return;
    }
    m_basePivot.setTargetAngle(ArmPose.getBasePivotAngle());
    m_extender.setTargetLength(ArmPose.getExtensionMeters());
    m_gripperPivot.setTargetAngle(ArmPose.getGripperPivotAngle());
    if ((!isPrep && isPoseReady()) || m_operator.rightBumper().getAsBoolean() || (DriverStation.isAutonomousEnabled() && isPoseClose() && m_stateTimer.hasElapsed(2))) {
      m_gripper.setCoralGripSpeed(0.5);
    } else {
      m_gripper.setCoralGripSpeed(0);
    }

    if (!isPrep && Robot.isSimulation() && getElapsedStateSeconds() > 2.0) {
      Gripper.hasCoralGrippedSim = false;
    }
  }

  private void prepClimb() {
    m_basePivot.setTargetAngle(ArmPoses.ClimbPrep.getBasePivotAngle());
    m_gripperPivot.setTargetAngle(ArmPoses.ClimbPrep.getGripperPivotAngle());
    m_extender.setTargetLength(ArmPoses.ClimbPrep.getExtensionMeters());
  }

  private void postClimb() {
    m_basePivot.setTargetAngle(ArmPoses.Climb.getBasePivotAngle());
    m_gripperPivot.setTargetAngle(ArmPoses.Climb.getGripperPivotAngle());
    m_extender.setTargetLength(ArmPoses.Climb.getExtensionMeters());
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
// RIP m_oldArm & m_oldGripper 2025-2025
