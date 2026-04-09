// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.commands.SparkPosition;
import frc.lib.commands.SparkSysIDTest;
import frc.lib.yams.units.EasyCRT;
import frc.lib.yams.units.EasyCRTConfig;
import frc.robot.CanIDs;
import frc.robot.Constants;

@Logged
public class TurretRotate extends SubsystemBase {

  public SparkMax rotateMotor;
  
  public EasyCRT crt;
  public EasyCRTConfig crtConfig;
  
  public TurretRotate() 
  {
    rotateMotor = new SparkMax(CanIDs.ROTATE_MOTOR_ID, MotorType.kBrushless);
    
    
    
    SparkMaxConfig rotateMotorConfig = new SparkMaxConfig();
      rotateMotorConfig
        .smartCurrentLimit(20)
        .idleMode(IdleMode.kBrake)
        .inverted(false);
      rotateMotorConfig.closedLoop
        .p(0.12025)
        .i(0.0)
        .d(0.0);
        
      rotateMotorConfig.closedLoop.feedForward.sva(0.20505, 0.0011331, 0.00010849);    
      rotateMotorConfig.softLimit
        .forwardSoftLimit(260)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(10)
        .reverseSoftLimitEnabled(true);
      rotateMotorConfig.signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .isAtSetpointAlwaysOn(true)
        .maxMotionSetpointPositionAlwaysOn(true)
        .maxMotionSetpointVelocityAlwaysOn(true)
        .setSetpointAlwaysOn(true);
      rotateMotorConfig.encoder
        .positionConversionFactor(360.0/33.16);
    
    rotateMotor.configure(rotateMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
    
    SmartDashboard.putData("Subsystem/TurretRotate",this);
    SmartDashboard.putData("SysID/Rotate",new SparkSysIDTest(rotateMotor, this, 2, -90, 90, rotateMotor.getEncoder()::getPosition));
    
  }

  @Override
  public void periodic() 
  {
    
    SmartDashboard.putNumber("Subsystem/TurretRotate/TargetAngle", rotateMotor.getClosedLoopController().getSetpoint());
    SmartDashboard.putNumber("Subsystem/TurretRotate/Actual", rotateMotor.getEncoder().getPosition());
  }

  public void point(double pos) 
  {
    rotateMotor.getClosedLoopController().setSetpoint(pos, ControlType.kPosition); // Gears   99.5:1
  } 

  

  


  /*
   * NUMBERS
   * 
   * Blue Hub Coordinates:
   *  x = 4.621
   *  y = 4.041
   * 
   * Red Hub Coordinates:
   *  x = 11.919
   *  y = 4.041
   */


}
