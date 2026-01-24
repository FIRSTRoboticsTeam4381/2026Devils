// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CanIDs;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  
  public SparkFlex intakePivotMotor;
  public SparkFlex intakeMotionMotor;

  public Intake() 
  {
    intakePivotMotor = new SparkFlex(CanIDs.INTAKE_PIVOT_MOTOR_ID, MotorType.kBrushless);
    intakeMotionMotor = new SparkFlex(CanIDs.INTAKE_MOTION_MOTOR_ID, MotorType.kBrushless);
  }

  @Override
  public void periodic() 
  {
    
  }
}
