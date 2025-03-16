// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.chaos131.gamepads.Gamepad;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Constants.ArmConstants.ArmPoses;
import frc.robot.Constants.ArmConstants.ExtenderConstants;
import frc.robot.Constants.ArmConstants.GripperConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Robot;
import frc.robot.subsystems.shared.StateBasedSubsystem;
import frc.robot.subsystems.shared.SubsystemState;
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
    public boolean hasAlgae;
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
    values.hasAlgae = m_gripper.hasAlgae();
    values.isGripperPivotAtSafeAngle = m_gripperPivot.isSafeAngle();
    return values;
  }

  public BasePivot m_basePivot = new BasePivot(this::getArmValues); // TODO: UNDO public
  public Extender m_extender = new Extender(this::getArmValues); // TODO: UNDO public
  public Gripper m_gripper = new Gripper(this::getArmValues);
  public GripperPivot m_gripperPivot = new GripperPivot(this::getArmValues);
  private Gamepad m_operator;
  private Gamepad m_driver;

  /**
   * The possible states of the Arm's state machine.
   */
  public enum ArmState implements SubsystemState {
    MANUAL,
    START,
    STOW,
    INTAKE_CORAL_FROM_FLOOR,
    INTAKE_ALGAE_FROM_FLOOR,
    INTAKE_FROM_HP, // Probably won't implement -Josh // nevermind -Josh
    PREP_L1,
    PREP_L2,
    PREP_L3,
    PREP_L4, // Might need many prep states
    PREP_PROCESSOR,
    PREP_BARGE,
    SCORE_L1,
    SCORE_L2,
    SCORE_L3,
    SCORE_L4,
    ALGAE_HIGH,
    ALGAE_LOW,
    SCORE_ALGAE,
    HOLD_CORAL,
    HOLD_ALGAE,
    PREP_CLIMB,
    POST_CLIMB;
  }

  /** Creates a new Arm. */
  public Arm(Gamepad operator, Gamepad driver) {
    super(ArmState.START);
    m_operator = operator;
    m_driver = driver;
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
      case INTAKE_CORAL_FROM_FLOOR:
        intakeCoralFromFloorState();
        break;
      case INTAKE_ALGAE_FROM_FLOOR:
        intakeAlgaeFromFloorState();
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
      case PREP_PROCESSOR:
        prepProcessorState();
        break;
      case PREP_BARGE:
        prepBargeState();
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
      case SCORE_ALGAE:
        scoreAlgaeState();
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
      case HOLD_ALGAE:
        holdAlgaeState();
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
      changeState(ArmState.STOW);
    } else {
      // changeState(ArmState.MANUAL);
      changeState(ArmState.STOW);
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
      changeState(ArmState.HOLD_CORAL);
      return;
    }
    if (m_gripper.hasAlgae()) {
      changeState(ArmState.HOLD_ALGAE);
      return;
    }
    m_basePivot.setTargetAngle(ArmPoses.Stow.getBasePivotAngle());
    m_extender.setTargetLength(ArmPoses.Stow.getExtensionMeters());
    m_gripperPivot.setTargetAngle(ArmPoses.Stow.getGripperPivotAngle());
    m_gripper.setCoralGripSpeed(0.0);
  }

  private void intakeCoralFromFloorState() {
    if (m_gripper.hasAlgae()) {
      m_gripper.setCoralGripSpeed(0.0);
      changeState(ArmState.HOLD_ALGAE);
      return;
    }

    if (!m_gripper.hasCoralFront()) {
      m_gripper.setCoralGripSpeed(GripperConstants.IntakeCoralSpeed); 
    } else if (m_gripper.hasCoralFront() && !m_gripper.hasCoralBack()) {
      m_gripper.setCoralGripSpeed(GripperConstants.IntakeCoralSlow);
    } else {
      m_gripper.setCoralGripSpeed(0.0);
      changeState(ArmState.HOLD_CORAL);
    }

    m_basePivot.setTargetAngle(ArmPoses.FloorIntakeCoral.getBasePivotAngle());
    m_extender.setTargetLength(ArmPoses.FloorIntakeCoral.getExtensionMeters());
    m_gripperPivot.setTargetAngle(ArmPoses.FloorIntakeCoral.getGripperPivotAngle());

    if (Robot.isSimulation() && getElapsedStateSeconds() > 2.0) {
      Gripper.hasCoralGrippedSim = true;
    }
  }

  private void intakeAlgaeFromFloorState() {
    if (!m_gripper.hasAlgae() && !m_gripper.hasCoral()) {
      m_gripper.setAlgaeGripSpeed(GripperConstants.IntakeAlgaeSpeed); 
    } else {
      m_gripper.setAlgaeGripSpeed(0.0);
      changeState(ArmState.HOLD_ALGAE);
    }

    m_basePivot.setTargetAngle(ArmPoses.FloorIntakeAlgae.getBasePivotAngle());
    m_extender.setTargetLength(ArmPoses.FloorIntakeAlgae.getExtensionMeters());
    m_gripperPivot.setTargetAngle(ArmPoses.FloorIntakeAlgae.getGripperPivotAngle());

    if (Robot.isSimulation() && getElapsedStateSeconds() > 2.0) {
      Gripper.hasAlgaeGrippedSim = true;
    }
  }

  private void intakeFromHpState() {
    if (m_gripper.hasAlgae()) {
      m_gripper.setCoralGripSpeed(0.0);
      changeState(ArmState.HOLD_ALGAE);
      return;
    }

    if (!m_gripper.hasCoralFront()) {
      m_gripper.setCoralGripSpeed(GripperConstants.IntakeCoralSpeed); 
    } else if (m_gripper.hasCoralFront() && !m_gripper.hasCoralBack()) {
      m_gripper.setCoralGripSpeed(GripperConstants.IntakeCoralSlow);
    } else {
      m_gripper.setCoralGripSpeed(0.0);
      changeState(ArmState.HOLD_CORAL);
      return;
    }

    m_basePivot.setTargetAngle(ArmPoses.HpIntake.getBasePivotAngle());
    m_extender.setTargetLength(ArmPoses.HpIntake.getExtensionMeters());
    m_gripperPivot.setTargetAngle(ArmPoses.HpIntake.getGripperPivotAngle());

    if (Robot.isSimulation() && getElapsedStateSeconds() > 2.0) {
      Gripper.hasCoralGrippedSim = true;
    }
  }

  private void prepL1State() {
    scoreHelperCoral(ArmPoses.ScoreL1, true);
  }

  private void prepL2State() {
    scoreHelperCoral(ArmPoses.ScoreL2, true);
  }

  private void prepL3State() {
    scoreHelperCoral(ArmPoses.ScoreL3, true);
  }

  private void prepL4State() {
    scoreHelperCoral(ArmPoses.ScoreL4, true);
  }

  private void prepProcessorState() {
    m_basePivot.setTargetAngle(ArmPoses.ScoreProcessor.getBasePivotAngle());
    m_extender.setTargetLength(ArmPoses.ScoreProcessor.getExtensionMeters());
    m_gripperPivot.setTargetAngle(ArmPoses.ScoreProcessor.getGripperPivotAngle());
  }

  private void prepBargeState() {
    m_basePivot.setTargetAngle(ArmPoses.ScoreBarge.getBasePivotAngle());
    m_extender.setTargetLength(ArmPoses.ScoreBarge.getExtensionMeters());
    m_gripperPivot.setTargetAngle(ArmPoses.ScoreBarge.getGripperPivotAngle());
  }

  private void scoreL1State() {
    scoreHelperCoral(ArmPoses.ScoreL1, false);
  }

  private void scoreL2State() {
    scoreHelperCoral(ArmPoses.ScoreL2, false);
  }

  private void scoreL3State() {
    scoreHelperCoral(ArmPoses.ScoreL3, false);
  }

  private void scoreL4State() {
    scoreHelperCoral(ArmPoses.ScoreL4, false);
  }

  private void scoreAlgaeState() {
    m_gripper.setAlgaeGripSpeed(GripperConstants.OutakeAlgaeSpeed);
    if (Robot.isSimulation() && getElapsedStateSeconds() > 2.0) {
      Gripper.hasAlgaeGrippedSim = false;
    }
  }

  private void algaeHighState() {
    if (!getArmValues().hasCoral) {
      m_basePivot.setTargetAngle(ArmPoses.AlgaeHigh.getBasePivotAngle());
      m_extender.setTargetLength(ArmPoses.AlgaeHigh.getExtensionMeters());
      m_gripperPivot.setTargetAngle(ArmPoses.AlgaeHigh.getGripperPivotAngle());
      m_gripper.setAlgaeGripSpeed(GripperConstants.IntakeAlgaeSpeed);
    } else {
      changeState(ArmState.HOLD_CORAL);
    }
  }

  private void algaeLowState() {
    if (!getArmValues().hasCoral) {
      m_basePivot.setTargetAngle(ArmPoses.AlgaeLow.getBasePivotAngle());
      m_extender.setTargetLength(ArmPoses.AlgaeLow.getExtensionMeters());
      m_gripperPivot.setTargetAngle(ArmPoses.AlgaeLow.getGripperPivotAngle());
      m_gripper.setAlgaeGripSpeed(GripperConstants.IntakeAlgaeSpeed);
    } else {
      changeState(ArmState.HOLD_CORAL);
    }
  }

  private void holdCoralState() {
    m_basePivot.setTargetAngle(ArmPoses.HoldCoral.getBasePivotAngle());
    m_extender.setTargetLength(ArmPoses.HoldCoral.getExtensionMeters());
    m_gripperPivot.setTargetAngle(ArmPoses.HoldCoral.getGripperPivotAngle());
    if (!m_gripper.hasCoral()) {
      changeState(ArmState.STOW);
      return;
    }
  }

  private void holdAlgaeState() {
    m_basePivot.setTargetAngle(ArmPoses.HoldAlgae.getBasePivotAngle());
    m_extender.setTargetLength(ArmPoses.HoldAlgae.getExtensionMeters());
    m_gripperPivot.setTargetAngle(ArmPoses.HoldAlgae.getGripperPivotAngle());
    if (!m_gripper.hasAlgae()) {
      changeState(ArmState.STOW);
      return;
    }
  }

  private void scoreHelperCoral(ArmPose armPose, boolean isPrep) {
    if (!(m_gripper.hasCoral())) {
      changeState(ArmState.STOW);
      return;
    }
    m_basePivot.setTargetAngle(armPose.getBasePivotAngle());
    m_extender.setTargetLength(armPose.getExtensionMeters());
    m_gripperPivot.setTargetAngle(armPose.getGripperPivotAngle());
    if ((!isPrep && isPoseReady()) || m_operator.rightBumper().getAsBoolean() || (DriverStation.isAutonomousEnabled() && isPoseClose() && m_stateTimer.hasElapsed(2))) {
      m_gripper.setCoralGripSpeed(GripperConstants.OutakeCoralSpeed);
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
    super.periodic();
    switch (getCurrentState()) {
      case INTAKE_CORAL_FROM_FLOOR:
      case INTAKE_ALGAE_FROM_FLOOR:
      case ALGAE_HIGH:
      case ALGAE_LOW:
      case INTAKE_FROM_HP:
        m_driver.getHID().setRumble(RumbleType.kBothRumble, GeneralConstants.RumbleIntensity);
        break;
    
      default:
        m_driver.getHID().setRumble(RumbleType.kBothRumble, 0);
        break;
    }
    Logger.recordOutput("CurrentState", getCurrentState().name());
  }
}
// RIP m_oldArm & m_oldGripper 2025-2025
