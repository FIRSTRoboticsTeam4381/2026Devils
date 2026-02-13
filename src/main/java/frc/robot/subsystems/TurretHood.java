// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.commands.SparkPosition;
import frc.robot.CanIDs;
import frc.robot.Constants;

@Logged
public class TurretHood extends SubsystemBase {

  public SparkMax hoodMotor;
  public InterpolatingDoubleTreeMap map;
  
  public TurretHood() 
  {
    map = new InterpolatingDoubleTreeMap();
    hoodMotor = new SparkMax(CanIDs.HOOD_MOTOR_ID, MotorType.kBrushless);
    setUpMap();
    SparkMaxConfig hoodMotorConfig = new SparkMaxConfig();
      hoodMotorConfig
        .advanceCommutation(60)
        .smartCurrentLimit(20)
        .idleMode(IdleMode.kBrake);
      hoodMotorConfig.closedLoop
        .p(0.001)
        .i(0.0)
        .d(0.0);
      hoodMotorConfig.closedLoop.feedForward.kV(0.0);    
      hoodMotorConfig.signals
        .absoluteEncoderPositionAlwaysOn(true)
        .isAtSetpointAlwaysOn(true)
        .maxMotionSetpointPositionAlwaysOn(true)
        .maxMotionSetpointVelocityAlwaysOn(true)
        .setSetpointAlwaysOn(true);
    SmartDashboard.putData("Subsystem/TurretHood",this);
  }

  public void setUpMap()
  {
    // map.put(0.0,0.0);
  }

  @Override
  public void periodic() 
  {
    
  }

  public Command hoodTo(double pos) 
  {
    return new SparkPosition(hoodMotor, pos, .5, this); // Change
  } 

  public void angleFromDist(double dist)
  {
    angle(map.get(dist));
  }

  public void angle(double pos)
  {
    hoodMotor.getClosedLoopController().setSetpoint(pos, ControlType.kPosition);
  }



  /*
   * NUMBERS
   * 
   * Blue Hub Coordinates:
   *  x = 4.621
   *  y = 4.041
   * 
   * Red Hub Coordinates:
   *  x = 11.919
   *  y = 4.041
   */


}
