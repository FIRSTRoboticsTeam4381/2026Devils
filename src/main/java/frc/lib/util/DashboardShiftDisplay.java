// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

/**
   * Utility class for showing hub/shift status on Elastic.
   */
public class DashboardShiftDisplay {
    /**
   * Update dashboard widgets displaying the current match period, time left,
   * and status of the red and blue hubs.
   */
  public static void updateHubStatus()
  {
    double mt = DriverStation.getMatchTime();

    if(mt == -1.0)
    {
      // Match time isn't set
      SmartDashboard.putString("MatchPeriod", "N/A");
      SmartDashboard.putNumber("PeriodTimer", mt);
      SmartDashboard.putStringArray("HubsActive", new String[]{"#000000"});
    }
    else if(DriverStation.isAutonomousEnabled())
    {
      SmartDashboard.putString("MatchPeriod", "Auto");
      SmartDashboard.putNumber("PeriodTimer", mt);
      SmartDashboard.putStringArray("HubsActive", new String[]{"#FF0000", "#000000", "#0000FF"});
    }
    else if(DriverStation.isTeleopEnabled())
    {
      // Figure out which match period this is
      if(mt <= 30)
      {
        SmartDashboard.putString("MatchPeriod", "Endgame");
        SmartDashboard.putNumber("PeriodTimer", mt);

        if(mt < 5)
          SmartDashboard.putStringArray("HubsActive", new String[]{
            blinkingColor(0.25, Color.kRed, Color.kBlack).toHexString(), 
            "#000000", 
            blinkingColor(0.25, Color.kBlue, Color.kBlack).toHexString()});
        else
          SmartDashboard.putStringArray("HubsActive", new String[]{"#FF0000", "#000000", "#0000FF"});
      }
      else if(mt <= 55) 
      {
        SmartDashboard.putString("MatchPeriod", "Shift 4");
        SmartDashboard.putNumber("PeriodTimer", mt - 30);
        updateHubForShift(4, mt-30);
      }
      else if(mt <= 80) 
      {
        SmartDashboard.putString("MatchPeriod", "Shift 3");
        SmartDashboard.putNumber("PeriodTimer", mt - 55);
        updateHubForShift(3, mt-55);
      }
      else if(mt <= 105) 
      {
        SmartDashboard.putString("MatchPeriod", "Shift 2");
        SmartDashboard.putNumber("PeriodTimer", mt - 80);
        updateHubForShift(2,mt-80);
      }
      else if(mt <= 130) 
      {
        SmartDashboard.putString("MatchPeriod", "Shift 1");
        SmartDashboard.putNumber("PeriodTimer", mt - 105);
        updateHubForShift(1,mt-105);
      }
      else
      {
        SmartDashboard.putString("MatchPeriod", "Transition");
        SmartDashboard.putNumber("PeriodTimer", mt - 130);

        if(DriverStation.getGameSpecificMessage().equals("R"))
          if(mt - 130 < 5)
            SmartDashboard.putStringArray("HubsActive", new String[]{
              blinkingColor(0.25, Color.kBlack, Color.kRed).toHexString(), "#000000", "#0000FF"});
          else
            SmartDashboard.putStringArray("HubsActive", new String[]{
              blinkingColor(0.8, Color.kRed, Color.kWhite).toHexString(), "#000000", "#0000FF"});
        else if(DriverStation.getGameSpecificMessage().equals("B"))
          if(mt - 130 < 5)
            SmartDashboard.putStringArray("HubsActive", new String[]{
              "#FF0000", "#000000", blinkingColor(0.25, Color.kBlack, Color.kBlue).toHexString()});
          else
            SmartDashboard.putStringArray("HubsActive", new String[]{
              "#FF0000", "#000000", blinkingColor(0.8,Color.kBlue, Color.kWhite).toHexString()});
        else
          SmartDashboard.putStringArray("HubsActive", new String[]{"#FF0000", "#000000", "#0000FF"});
      }

    }
  }

  private static void updateHubForShift(int shift, double timeLeft)
  {
    String gameData = DriverStation.getGameSpecificMessage();

    // Figure out whose turn it is
    boolean redEven = gameData.equals("R");
    boolean blueEven = gameData.equals("B");

    boolean redsTurn = redEven && shift%2==0 || blueEven && shift%2==1;
    boolean bluesTurn = blueEven && shift%2==0 || redEven && shift%2==1;

    if(redsTurn)
      if(timeLeft < 5)
        SmartDashboard.putStringArray("HubsActive", new String[]{
          blinkingColor(0.25, Color.kRed, Color.kBlack).toHexString(), "#000000", "#000000"});
      else
        SmartDashboard.putStringArray("HubsActive", new String[]{"#FF0000", "#000000", "#000000"});
    else if(bluesTurn)
      if(timeLeft < 5)
        SmartDashboard.putStringArray("HubsActive", new String[]{
          "#000000", "#000000", blinkingColor(0.25, Color.kBlue, Color.kBlack).toHexString()});
      else
        SmartDashboard.putStringArray("HubsActive", new String[]{"#000000", "#000000", "#0000FF"});
    else
      SmartDashboard.putStringArray("HubsActive", new String[]{"#000000"});
  }

  private static Color blinkingColor(double periodS, Color color1, Color color2)
  {
    double blink = Math.abs( (Timer.getTimestamp() % periodS) - 0.5*periodS) / (0.5*periodS+0.0001);
    return Color.lerpRGB(color1, color2, blink);
  }

}
