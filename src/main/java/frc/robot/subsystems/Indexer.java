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
  
  
  public Indexer() 
  {
    indexerMotor = new SparkMax(CanIDs.INDEX_MOTOR_ID, MotorType.kBrushless);
    

    this.setDefaultCommand(indexerOff());

    SparkMaxConfig indexerMotorConfig = new SparkMaxConfig();
      indexerMotorConfig
      .smartCurrentLimit(80)
      .idleMode(IdleMode.kBrake)
      .inverted(true);

    
    indexerMotorConfig.signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .isAtSetpointAlwaysOn(true)
        .maxMotionSetpointPositionAlwaysOn(true)
        .maxMotionSetpointVelocityAlwaysOn(true)
        .setSetpointAlwaysOn(true);
    

    indexerMotor.configure(indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    SmartDashboard.putData("Subsystem/Indexer",this);
  }

  @Override
  public void periodic() 
  {
    
  }

  // TODO add commands
  public Command indexerThrough()
  {
    return new InstantCommand(() -> indexerMotor.set(.8),this).repeatedly();
  }
  public Command indexerBack()
  {
    return new InstantCommand(() -> indexerMotor.set(-.8),this).repeatedly();
  }

  public Command indexerIdle()
  {
    return new InstantCommand(() -> indexerMotor.set(-.05),this).repeatedly();
  }

  public Command indexerOff()
  {
    return new InstantCommand(() -> indexerMotor.set(0),this);
  }

}
