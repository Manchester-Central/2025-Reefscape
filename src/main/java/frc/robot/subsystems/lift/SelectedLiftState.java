// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import frc.robot.subsystems.lift.IdLift.LiftState;

/** Add your docs here. */
public enum SelectedLiftState {
  L1(LiftState.PREP_L1, LiftState.SCORE_L1),
  L2(LiftState.PREP_L2, LiftState.SCORE_L2),
  L3(LiftState.PREP_L3, LiftState.SCORE_L3),
  L4(LiftState.PREP_L4, LiftState.SCORE_L4);

  public final LiftState PrepState;
  public final LiftState ScoreState;

  private SelectedLiftState(LiftState prepState, LiftState scoreState) {
    PrepState = prepState;
    ScoreState = scoreState;
  }
}
