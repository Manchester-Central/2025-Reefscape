// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import org.littletonrobotics.junction.Logger;

import com.chaos131.gamepads.Gamepad;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.FieldDimensions;
import frc.robot.Constants.MidLiftConstants.LiftPoses;
import frc.robot.Robot;
import frc.robot.subsystems.shared.StateBasedSubsystem;
import frc.robot.subsystems.shared.SubsystemState;
import frc.robot.utils.FieldPoint;
import frc.robot.utils.IkEquations;
import java.util.function.Supplier;

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
  private Gripper m_gripper = new Gripper(this::getLiftValues);
  private GripperPivot m_gripperPivot = new GripperPivot(this::getLiftValues);
  private Gamepad m_operator;
  private Pose3d m_ikTargetPose = new Pose3d(0, 0, FieldDimensions.Reef3Meters, new Rotation3d());
  private Supplier<Pose2d> m_robotPoseSupplier;

  /**
   * The possible states of the Lift's state machine.
   */
  public enum LiftState implements SubsystemState {
    MANUAL,
    START,
    STOW,
    IKSOLVER,
    INTAKE_FROM_FLOOR,
    INTAKE_FROM_HP, // Probably won't implement -Josh // nevermind -Josh
    SCORE_L1,
    SCORE_L2,
    SCORE_L3,
    SCORE_L4, // Might need many prep states
    HOLD_CORAL,
    BOTTOM_BUCKET,
    TOP_BUCKET;
  }

  /** Creates a new Lift. */
  public IdLift(Gamepad operator, Supplier<Pose2d> robotPose) {
    super(LiftState.START);
    m_operator = operator;
    m_robotPoseSupplier = robotPose;
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
      case IKSOLVER:
        ikSolverState();
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
      case HOLD_CORAL:
        holdCoralState();
        break;
      case BOTTOM_BUCKET:
        bottombucketState();
        break;
      case TOP_BUCKET:
        topbucketState();
        break;
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
    Transform3d branchLeftR4 = new Transform3d(FieldDimensions.ReefBranchLeft.getX(),
                                               FieldDimensions.ReefBranchLeft.getY(),
                                               FieldDimensions.Reef3Meters,
                                               new Rotation3d(0, -Math.PI / 2, 0));
    var reefBranch = FieldPoint.aprilTagMap.get(17).pose3d.transformBy(branchLeftR4);
    
    LiftPose mechanismPose = IkEquations.getPivotLiftPivot(robotPose, reefBranch);

    // Now apply them!
    m_extender.setTargetLength(mechanismPose.getExtensionMeters());
    m_basePivot.setTargetAngle(mechanismPose.getBasePivotAngle());
    m_gripperPivot.setTargetAngle(mechanismPose.getGripperPivotAngle());
  }

  private void startState() {
    if (Robot.isSimulation()) {
      // changeState(LiftState.STOW);
      changeState(LiftState.IKSOLVER);
      // changeState(LiftState.MANUAL);
    } else {
      changeState(LiftState.MANUAL);
    }
  }

  private void manualState() {
    /* m_basePivot.setSpeed(m_operator.getLeftY() * 0.131);
    m_gripper.setCoralGripSpeed(0.0);
    m_gripperPivot.setSpeed(0.0);
    m_extender.setSpeed(m_operator.getRightY() * 0.5);
    */
    // setMotorStartUp();
    m_basePivot.setTargetAngle(Rotation2d.fromDegrees(90));
    if (m_extender.getCurrentLength() >= LiftPoses.HoldCoral.getExtensionMeters()) {
      // for operator controls
      m_gripperPivot.setSpeed(m_operator.getLeftY() * 0.4);
      double yValue = m_operator.getRightY();
      if (m_gripper.hasCoral()) {
        yValue = yValue < 0 ? 0 : yValue;
      }
      m_gripper.setCoralGripSpeed(yValue * 0.131);
    } else {
      m_gripperPivot.setTargetAngle(Rotation2d.fromDegrees(0));
      if (m_gripperPivot.atTarget()) {
        setMotorCleanUp(); // TODO Command Scheduler Loop overun
        m_extender.setTargetLength(LiftPoses.HoldCoral.getExtensionMeters());
      }
    }
  }

  private void stowState() {
    if (m_gripper.hasCoral()) {
      changeState(LiftState.HOLD_CORAL);
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

  private void holdCoralState() {
    m_basePivot.setTargetAngle(LiftPoses.HoldCoral.getBasePivotAngle());
    m_extender.setTargetLength(LiftPoses.HoldCoral.getExtensionMeters());
    m_gripperPivot.setTargetAngle(LiftPoses.HoldCoral.getGripperPivotAngle());
    if (!m_gripper.hasCoral()) {
      changeState(LiftState.TOP_BUCKET);
      return;
    }
  }

  private void bottombucketState() {
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

  private void topbucketState() {
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

  private void scoreHelper(LiftPose liftPose, boolean isGripperReleaseForward) {
    if (!(m_gripper.hasCoral())) {
      changeState(LiftState.STOW);
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

    if (Robot.isSimulation() && getElapsedStateSeconds() > 2.0) {
      Gripper.hasCoralGrippedSim = false;
    }
  }

  /**
   * Set Motor to clean up. :3
   */
  public void setMotorCleanUp() {
    m_extender.setMotorCoast();
    m_basePivot.setMotorCoast();
    m_gripperPivot.setMotorCoast();
  }

  /**
   * Set Motor to start up. :3
   */
  public void setMotorStartUp() {
    m_extender.setMotorBrake();
    m_basePivot.setMotorBrake();
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
