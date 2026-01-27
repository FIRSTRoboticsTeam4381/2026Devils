// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CanIDs;
import frc.robot.Constants;

public class TurretHood extends SubsystemBase {

  public SparkMax hoodMotor;
  
  public TurretHood() 
  {
    hoodMotor = new SparkMax(CanIDs.HOOD_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig hoodMotorConfig = new SparkMaxConfig();
      hoodMotorConfig
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kBrake)
        ;
  }

  @Override
  public void periodic() 
  {
    
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
