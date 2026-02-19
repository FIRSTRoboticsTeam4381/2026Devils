// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CanIDs;
import frc.robot.Constants;

@Logged
public class Indexer extends SubsystemBase {

  public SparkMax indexerMotor;
  public SparkFlex pushMotor;
  
  public Indexer() 
  {
    indexerMotor = new SparkMax(CanIDs.INDEX_MOTOR_ID, MotorType.kBrushless);
    pushMotor = new SparkFlex(CanIDs.PUSH_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig indexerMotorConfig = new SparkMaxConfig();
      indexerMotorConfig
      .smartCurrentLimit(15)
      .idleMode(IdleMode.kBrake)
      .advanceCommutation(60);

    SparkFlexConfig pushMotorConfig = new SparkFlexConfig();
      pushMotorConfig
      .smartCurrentLimit(80)
      .idleMode(IdleMode.kBrake)
      .follow(indexerMotor,true);
    
    indexerMotorConfig.signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .isAtSetpointAlwaysOn(true)
        .maxMotionSetpointPositionAlwaysOn(true)
        .maxMotionSetpointVelocityAlwaysOn(true)
        .setSetpointAlwaysOn(true);
    pushMotorConfig.signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .isAtSetpointAlwaysOn(true)
        .maxMotionSetpointPositionAlwaysOn(true)
        .maxMotionSetpointVelocityAlwaysOn(true)
        .setSetpointAlwaysOn(true);

    indexerMotor.configure(indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pushMotor.configure(pushMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SmartDashboard.putData("Subsystem/Indexer",this);
  }

  @Override
  public void periodic() 
  {
    
  }

  // TODO add commands
  public Command pushOn()
  {
    return new InstantCommand(() -> indexerMotor.set(.5),this).repeatedly();
  }

  public Command pushOff()
  {
    return new InstantCommand(() -> indexerMotor.set(0),this);
  }

}
