// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CanIDs;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

  public SparkFlex indexerMotor;
  public SparkFlex pushMotor;
  
  public Indexer() 
  {
    indexerMotor = new SparkFlex(CanIDs.INDEX_MOTOR_ID, MotorType.kBrushless);
    pushMotor = new SparkFlex(CanIDs.PUSH_MOTOR_ID, MotorType.kBrushless);

    SparkFlexConfig indexerMotorConfig = new SparkFlexConfig();
      indexerMotorConfig
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake);

    SparkFlexConfig pushMotorConfig = new SparkFlexConfig();
      pushMotorConfig
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake)
      .follow(indexerMotor);

  }

  @Override
  public void periodic() 
  {
    
  }

  // TODO add commands
  public Command pushOn()
  {
    return new InstantCommand(() -> indexerMotor.set(0.5));
  }

  public Command pushOff()
  {
    return new InstantCommand(() -> indexerMotor.set(0));
  }

}
