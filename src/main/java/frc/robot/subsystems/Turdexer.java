// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.commands.SparkSysIDTest;
import frc.robot.CanIDs;

public class Turdexer extends SubsystemBase 
{
  public SparkFlex lateralTurdexerMotor;
  public SparkFlex medialTurdexerMotor;

  public Turdexer() 
  {
    lateralTurdexerMotor = new SparkFlex(CanIDs.LATERAL_TURDEXER_MOTOR_ID, MotorType.kBrushless);
    medialTurdexerMotor = new SparkFlex(CanIDs.MEDIAL_TURDEXER_MOTOR_ID, MotorType.kBrushless);
    this.setDefaultCommand(turdexerOff());

    SparkFlexConfig lateralTurdexerMotorConfig = new SparkFlexConfig();
    SparkFlexConfig medialTurdexerMotorConfig = new SparkFlexConfig();
      lateralTurdexerMotorConfig
      .smartCurrentLimit(60)
      .idleMode(IdleMode.kBrake)
      .inverted(true)
      .closedLoop
        .p(5.2496E-07)
        .i(0)
        .d(0)
        .feedForward
        .sva(0.36262, 0.0018211, 0.00011781);
      

      lateralTurdexerMotorConfig.signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .isAtSetpointAlwaysOn(true)
        .maxMotionSetpointPositionAlwaysOn(true)
        .maxMotionSetpointVelocityAlwaysOn(true)
        .setSetpointAlwaysOn(true);
      medialTurdexerMotorConfig
      .apply(lateralTurdexerMotorConfig)
      .follow(lateralTurdexerMotor);

      lateralTurdexerMotor.configure(lateralTurdexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      medialTurdexerMotor.configure(medialTurdexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      SmartDashboard.putData("SysID/Turdexer", new SparkSysIDTest(lateralTurdexerMotor, this, 2));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

   public void turdexerSpeed(Double vel)
  {
    lateralTurdexerMotor.getClosedLoopController().setSetpoint(vel, ControlType.kVelocity);
  }

  public Command turdexerThrough()
  {
    return new InstantCommand(() -> turdexerSpeed(3100.0),this).repeatedly();
  }
  public Command turdexerBack()
  {
    return new InstantCommand(() -> turdexerSpeed(-1500.0),this).repeatedly();
  }

  public Command turdexerOff()
  {
    return new InstantCommand(() -> lateralTurdexerMotor.set(0),this);
  }
}
