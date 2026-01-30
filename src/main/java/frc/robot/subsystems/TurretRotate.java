// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.commands.SparkPosition;
import frc.robot.CanIDs;
import frc.robot.Constants;

public class TurretRotate extends SubsystemBase {

  public SparkMax rotateMotor;
  
  public TurretRotate() 
  {
    rotateMotor = new SparkMax(CanIDs.ROTATE_MOTOR_ID, MotorType.kBrushless);
    
    SparkMaxConfig rotateMotorConfig = new SparkMaxConfig();
      rotateMotorConfig
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kBrake);
      rotateMotorConfig.closedLoop
        .p( 0.001)
        .i(0.0)
        .d(0.0);
      rotateMotorConfig.closedLoop.feedForward.kV(0.0);    
  }

  @Override
  public void periodic() 
  {
    
  }

  public Command rotateTo(double pos) 
  {
    return new SparkPosition(rotateMotor, pos, .5, this); // Change
  } 

  public void angle(double pos)
  {
    rotateTo(pos);
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
