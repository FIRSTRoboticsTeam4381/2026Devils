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
import frc.lib.commands.SparkPositionProfiled;
import frc.lib.commands.SparkSysIDTest;
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
    this.setDefaultCommand(intakeStop());

    SparkFlexConfig intakePivotConfig = new SparkFlexConfig();
      intakePivotConfig
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake);
      intakePivotConfig.closedLoop
        .p(4.1815)
        .i(0)
        .d(0.0031332)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .feedForward
        .sva(0.72818, 0.09364, 0);
        
      intakePivotConfig.softLimit
      .reverseSoftLimit(.2)
      .forwardSoftLimit(.5)
      .reverseSoftLimitEnabled(true)
      .forwardSoftLimitEnabled(true);
      
      

    SparkMaxConfig intakeMotionConfig = new SparkMaxConfig();
      intakeMotionConfig
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake);

    intakeMotionMotor.configure(intakeMotionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakePivotMotor.configure(intakePivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putData("SysID/Intake", new SparkSysIDTest(intakePivotMotor, this, 2,.220,.5,encoder::getPosition));
    SmartDashboard.putData("Subsystem/Intake",this);

    
  }

  @Override
  public void periodic() 
  {
    
  }

  public Command intakeIn()
  {
    return new InstantCommand(() -> intakeMotionMotor.set(0.80), this).repeatedly();
  }
  
  public Command intakeOut()
  {
    return new InstantCommand(() -> intakeMotionMotor.set(-0.80), this).repeatedly();
  }

  public Command intakeStop()
  {
    return new InstantCommand(() -> intakeMotionMotor.set(0), this);
  }

  public Command intakePivotStop()
  {
    return new InstantCommand(() -> intakePivotMotor.set(0), this);
  }

  public Command intakePivotTo(double position)
  {
    return new SparkPosition(intakePivotMotor, position, .015, this);
  }

  public Command intakeUndeploy()
  {
    return new SequentialCommandGroup(
      intakeStop(),
      intakePivotTo(.256),
      intakePivotStop()  
    );
  }

  public Command intakeDeploy() // TODO automatically turn this on
  {
    return new SequentialCommandGroup(
      intakePivotTo(.37),
      intakePivotStop()
    );
  }

  

  

}
