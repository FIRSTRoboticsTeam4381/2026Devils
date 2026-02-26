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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CanIDs;

public class Turdexer extends SubsystemBase 
{
  public SparkFlex turdexerMotor;

  public Turdexer() 
  {
    turdexerMotor = new SparkFlex(CanIDs.TURDEXER_MOTOR_ID, MotorType.kBrushless);
    this.setDefaultCommand(turdexerOff());

    SparkFlexConfig turdexerMotorConfig = new SparkFlexConfig();
      turdexerMotorConfig
      .smartCurrentLimit(80)
      .idleMode(IdleMode.kBrake)
      .inverted(true);

      turdexerMotorConfig.signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .isAtSetpointAlwaysOn(true)
        .maxMotionSetpointPositionAlwaysOn(true)
        .maxMotionSetpointVelocityAlwaysOn(true)
        .setSetpointAlwaysOn(true);

      turdexerMotor.configure(turdexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command turdexerOn()
  {
    return new InstantCommand(() -> turdexerMotor.set(.5),this).repeatedly();
  }

  public Command turdexerOff()
  {
    return new InstantCommand(() -> turdexerMotor.set(0),this);
  }
}
