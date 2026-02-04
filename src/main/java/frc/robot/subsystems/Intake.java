// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.commands.SparkPosition;
import frc.robot.CanIDs;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  
  public SparkFlex intakePivotMotor;
  public SparkFlex intakeMotionMotor;

  public Boolean intakeStatus;

  public Intake() 
  {
    intakePivotMotor = new SparkFlex(CanIDs.INTAKE_PIVOT_MOTOR_ID, MotorType.kBrushless);
    intakeMotionMotor = new SparkFlex(CanIDs.INTAKE_MOTION_MOTOR_ID, MotorType.kBrushless);

    this.setDefaultCommand(intakeResting());

    SparkFlexConfig intakePivotConfig = new SparkFlexConfig();
      intakePivotConfig
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake);

    SparkFlexConfig intakeMotionConfig = new SparkFlexConfig();
      intakeMotionConfig
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() 
  {
    
  }

  public Command intake()
  {
    return new InstantCommand(() -> intakeMotionMotor.set(0.5), this);
  }

  public Command intakeStop()
  {
    return new InstantCommand(() -> intakeMotionMotor.set(0), this);
  }

  public Command intakePivotTo(double position)
  {
    return new SparkPosition(intakePivotMotor, position, 1, this);
  }

  public Command intakeResting()
  {
    return new ParallelCommandGroup(
      intakeStop(),
      intakePivotTo(0) // Position may need adjustment 
    );
  }

  public Command intakeReady() // TODO automatically turn this on
  {
    return new ParallelCommandGroup(
      intake(),
      intakePivotTo(0) // Change position to intaking ready position - need robot ):
    );
  }

}
