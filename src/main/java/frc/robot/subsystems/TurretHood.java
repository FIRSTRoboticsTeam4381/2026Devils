// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.w3c.dom.views.DocumentView;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.commands.SparkPosition;
import frc.lib.commands.SparkSysIDTest;
import frc.robot.CanIDs;
import frc.robot.Constants;
import frc.robot.commands.GeekedSysID;

@Logged
public class TurretHood extends SubsystemBase {

  public SparkMax hoodMotor;
  public InterpolatingDoubleTreeMap map;
  public AbsoluteEncoder encoder;
  
  
  public TurretHood() 
  {
    

    map = new InterpolatingDoubleTreeMap();
    hoodMotor = new SparkMax(CanIDs.HOOD_MOTOR_ID, MotorType.kBrushless);
    encoder = hoodMotor.getAbsoluteEncoder();
    setUpMap();
    SparkMaxConfig hoodMotorConfig = new SparkMaxConfig();
      hoodMotorConfig
        .advanceCommutation(60)
        .smartCurrentLimit(20)
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        ;
      hoodMotorConfig.closedLoop
        .p(2.0789)
        .i(0.0)
        .d(0.0)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .feedForward.sva(0.14567, 0.83706, 0.076267);
      hoodMotorConfig.closedLoop.feedForward.kV(0.0);    
      hoodMotorConfig.signals
        .absoluteEncoderPositionAlwaysOn(true)
        .isAtSetpointAlwaysOn(true)
        .maxMotionSetpointPositionAlwaysOn(true)
        .maxMotionSetpointVelocityAlwaysOn(true)
        .setSetpointAlwaysOn(true);
      hoodMotorConfig.softLimit
      .forwardSoftLimit(.95).reverseSoftLimit(.05)
      .forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);
      hoodMotorConfig.absoluteEncoder
        .inverted(true);

    hoodMotor.configure(hoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putData("SysID/Hood", new GeekedSysID(hoodMotor, this, 2,0.4,0.75, encoder::getPosition, new SysIdRoutine.Config()));
    SmartDashboard.putData("Subsystem/TurretHood",this);

    SmartDashboard.putNumber("Subsystem/TurretHood/HoodPos", 0.0);
    SmartDashboard.putData("Subsystem/TurretHood/HoodPosSet", new InstantCommand(()->angle(SmartDashboard.getNumber("Subsystem/TurretHood/HoodPos", 0.0)),this).repeatedly());
  }

  public void setUpMap()
  {
     map.put(2.1,0.05);
     map.put(3.15,0.10);
     map.put(3.4,0.15);
     map.put(4.8,0.37);
     map.put(5.5, 0.34);
     map.put(8.0, 0.45);
  }

  @Override
  public void periodic() 
  {
    SmartDashboard.putNumber("Subsystem/TurretHood/CurHoodPos", encoder.getPosition());
  }
  
  public void joystickCont(double speed)
  {
    hoodMotor.set(speed);
  }

  public Command hoodTo(double pos) 
  {
    return new SparkPosition(hoodMotor, pos, .5, this); // Change
  } 

  public void angleFromDist(double dist)
  {
    angle(map.get(dist));
  }

  public void angle(double pos)
  {
    hoodMotor.getClosedLoopController().setSetpoint(pos, ControlType.kPosition);
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
