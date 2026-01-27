// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CanIDs;
import frc.robot.Constants;

public class TurretShoot extends SubsystemBase {

  public SparkMax shootMotor1;
  public SparkMax shootMotor2;
  

  public TurretShoot() 
  {
    shootMotor1 = new SparkMax(CanIDs.SHOOT_MOTOR_1_ID, MotorType.kBrushless);
    shootMotor2 = new SparkMax(CanIDs.SHOOT_MOTOR_2_ID, MotorType.kBrushless);

    SparkMaxConfig shootMotor1Config = new SparkMaxConfig();
      shootMotor1Config
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kBrake)
        ;
    
    SparkMaxConfig shootMotor2Config = new SparkMaxConfig();
      shootMotor2Config
        .apply(shootMotor1Config)
        .follow(shootMotor1,true)
        .inverted(true)
        ;
  }

  @Override
  public void periodic() 
  {

  }

  public Command shootOn()
  {
    return new InstantCommand(() -> shootMotor1.set(.5));
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
