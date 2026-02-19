// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.commands.SparkPosition;
import frc.robot.CanIDs;
import frc.robot.Constants;

@Logged
public class Climb extends SubsystemBase {

  public SparkFlex climbPivotMotor;
  public SparkFlex climbElevatorMotor1;
  public SparkFlex climbElevatorMotor2;

  public Climb() 
  {
    climbPivotMotor = new SparkFlex(CanIDs.CLIMB_PIVOT_MOTOR_ID, MotorType.kBrushless);
    climbElevatorMotor1 = new SparkFlex(CanIDs.CLIMB_ELEVATOR_MOTOR_1_ID, MotorType.kBrushless);
    climbElevatorMotor2 = new SparkFlex(CanIDs.CLIMB_ELEVATOR_MOTOR_2_ID, MotorType.kBrushless);

    SparkFlexConfig climbPivot1Config = new SparkFlexConfig();
      climbPivot1Config
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kBrake)
        ;

    SparkFlexConfig climbElevator1Config = new SparkFlexConfig();
      climbElevator1Config
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kBrake)
        ;
    
    SparkFlexConfig climbElevator2Config = new SparkFlexConfig();
      climbElevator2Config
        .apply(climbElevator1Config)
        .follow(climbElevatorMotor1,true)
        ;
        

      climbPivotMotor.configure(climbPivot1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      climbElevatorMotor1.configure(climbElevator1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      climbElevatorMotor2.configure(climbElevator2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SmartDashboard.putData("Subsystem/Climb",this);
  }

  @Override
  public void periodic() 
  {
    
  }

  public Command climbPivotTo(double distance) 
  {
    return new SparkPosition(climbPivotMotor, distance, 1.0, this);
  } 
  
  public Command climbElevatorTo(double distance) 
  {
    return new SparkPosition(climbElevatorMotor1, distance, 2.0, this);
  } 
  
  public Command prepPivot()
  {
    return climbPivotTo(0).withName("Prep Climb Pivot");
  } 
  
  public Command prepElevator()
  {
    return climbElevatorTo(0).withName("Prep Climb Elevator");
  }
}
