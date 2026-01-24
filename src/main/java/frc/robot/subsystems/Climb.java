// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CanIDs;
import frc.robot.Constants;

public class Climb extends SubsystemBase {

  public SparkFlex climbPivotMotor;
  public SparkFlex climbElevatorMotor1;
  public SparkFlex climbElevatorMotor2;

  public Climb() 
  {
    climbPivotMotor = new SparkFlex(CanIDs.CLIMB_PIVOT_MOTOR_ID, MotorType.kBrushless);
    climbElevatorMotor1 = new SparkFlex(CanIDs.CLIMB_ELEVATOR_MOTOR_1_ID, MotorType.kBrushless);
    climbElevatorMotor2 = new SparkFlex(CanIDs.CLIMB_ELEVATOR_MOTOR_2_ID, MotorType.kBrushless);
  }

  @Override
  public void periodic() 
  {
    
  }
}
