// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.ArmConstants.GripperPivotConstants;
import java.util.List;

/** Add your docs here. */
public class SafetyUtil {
  /**
   * A class to manage our min/max angles for a given extender range.
   */
  public static class GripperPivotSafety {
    private final Distance m_distanceHigh;
    private final Distance m_distanceLow;
    private final Angle m_minAngle;
    private final Angle m_maxAngle;
    
    /**
     * Creates a new Gripper Pivot Safety.
     */
    public GripperPivotSafety(Distance distanceLow, Distance distanceHigh, Angle minAngle, Angle maxAngle)  {
      m_distanceLow = distanceLow;
      m_distanceHigh = distanceHigh;
      m_minAngle = minAngle;
      m_maxAngle = maxAngle;
    }

    public Angle getMaxAngle() {
      return m_maxAngle;
    }

    public Angle getMinAngle() {
      return m_minAngle;
    }

    public Distance getDistanceLow() {
      return m_distanceLow;
    }

    public Distance getDistanceHigh() {
      return m_distanceHigh;
    }

    /**
     * Gets the min/max angle for the given extender distance.
     */
    public static GripperPivotSafety getGripperPivotSafety(Distance currentDistance, List<GripperPivotSafety> safeties) {
      for (GripperPivotSafety gripperPivotSafety : safeties) {
        if (currentDistance.gte(gripperPivotSafety.m_distanceLow) && currentDistance.lte(gripperPivotSafety.m_distanceHigh)) {
          return gripperPivotSafety;
        }
      }
      return new GripperPivotSafety(currentDistance, currentDistance, GripperPivotConstants.TrueSafeAngle, GripperPivotConstants.TrueSafeAngle);
    }
  }
}
