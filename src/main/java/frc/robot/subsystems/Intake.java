// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.commands.SparkPosition;
import frc.robot.CanIDs;
import frc.robot.Constants;

@Logged
public class Intake extends SubsystemBase {
  
  public SparkFlex intakePivotMotor;
  public SparkMax intakeMotionMotor;
  public AbsoluteEncoder encoder;

  public Boolean intakeStatus;

  public Intake() 
  {
    intakePivotMotor = new SparkFlex(CanIDs.INTAKE_PIVOT_MOTOR_ID, MotorType.kBrushless);
    intakeMotionMotor = new SparkMax(CanIDs.INTAKE_MOTION_MOTOR_ID, MotorType.kBrushless);
    encoder = intakePivotMotor.getAbsoluteEncoder();
    this.setDefaultCommand(intakeResting());

    SparkFlexConfig intakePivotConfig = new SparkFlexConfig();
      intakePivotConfig
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake);
      intakePivotConfig.closedLoop
        .p(0.001)
        .i(0)
        .d(0)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    SparkMaxConfig intakeMotionConfig = new SparkMaxConfig();
      intakeMotionConfig
      .smartCurrentLimit(20)
      .idleMode(IdleMode.kBrake)
      .advanceCommutation(60)
      .inverted(true);

    intakeMotionMotor.configure(intakeMotionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakePivotMotor.configure(intakePivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SmartDashboard.putData("Subsystem/Intake",this);
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
    return new SequentialCommandGroup(
      intakeStop()//,
      //intakePivotTo(.2) // Position may need adjustment 
    );
  }

  public Command intakeReady() // TODO automatically turn this on
  {
    return new SequentialCommandGroup(
      
      //intakePivotTo(.5), // Change position to intaking ready position - need robot ):
      intake().repeatedly()
    );
  }

}
