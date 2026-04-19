// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.commands.SparkSysIDTest;
import frc.robot.CanIDs;
import frc.robot.Constants;

@Logged
public class Indexer extends SubsystemBase {

  public SparkFlex indexerMotor;
  public static double agitateRatio;
  
  public Indexer() 
  {
    indexerMotor = new SparkFlex(CanIDs.INDEX_MOTOR_ID, MotorType.kBrushless);
    agitateRatio = 1;

    this.setDefaultCommand(indexerOff());

    SparkMaxConfig indexerMotorConfig = new SparkMaxConfig();
      indexerMotorConfig
      .smartCurrentLimit(60)
      .idleMode(IdleMode.kBrake)
      .inverted(true)
      .closedLoop
        .p(1.0783E-06)
        .i(0)
        .d(0)
        .feedForward
        .sva(0.11833, 0.0023319, 0.00031664);

    
    indexerMotorConfig.signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .isAtSetpointAlwaysOn(true)
        .maxMotionSetpointPositionAlwaysOn(true)
        .maxMotionSetpointVelocityAlwaysOn(true)
        .setSetpointAlwaysOn(true);
    

    indexerMotor.configure(indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    SmartDashboard.putData("SysID/Indexer", new SparkSysIDTest(indexerMotor, this, 2));

    SmartDashboard.putData("Subsystem/Indexer",this);
  }

  @Override
  public void periodic() 
  {
    SmartDashboard.putNumber("Subsystem/Indexer/AgitateRatio", agitateRatio);
  }

  // TODO add commands

  public void indexerSpeed(Double vel)
  {
    indexerMotor.getClosedLoopController().setSetpoint(vel, ControlType.kVelocity);
  }

  public Command indexerThrough()
  {
    return new SequentialCommandGroup(
      new InstantCommand(() -> indexerSpeed(-1000.0),this),
      new WaitCommand(.05*agitateRatio),
      new InstantCommand(() -> indexerSpeed(1000.0),this),
      new WaitCommand(.25*1+1-agitateRatio)).repeatedly();
  }
  public Command agitateInc()
  {
    return new InstantCommand(()-> agitateRatio = agitateRatio +.1);
  }
  public Command agitateDec()
  {
    return new InstantCommand(()-> agitateRatio = agitateRatio -.1);
  }

  public Command indexerNoJitter()
  {
    return new InstantCommand(() -> indexerSpeed(1000.0),this);
  }
  public Command indexerBack()
  {
    return new InstantCommand(() -> indexerSpeed(-1000.0),this).repeatedly();
  }

  public Command indexerIdle()
  {
    return new InstantCommand(() -> indexerMotor.set(-.05),this).repeatedly();
  }

  public Command indexerOff()
  {
    return new InstantCommand(() -> indexerSpeed(0.0),this);
  }

}
