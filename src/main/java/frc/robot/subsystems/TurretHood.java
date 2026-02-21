// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.commands.SparkPosition;
import frc.lib.commands.SparkSysIDTest;
import frc.robot.CanIDs;
import frc.robot.Constants;

@Logged
public class TurretHood extends SubsystemBase {

  public SparkMax hoodMotor;
  public InterpolatingDoubleTreeMap map;
  public AbsoluteEncoder encoder;
  //public double goTo;
  
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
        .inverted(true)
        .absoluteEncoder.inverted(true);
      hoodMotorConfig.closedLoop
        .p(0.001)
        .i(0.0)
        .d(0.0)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
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

    hoodMotor.configure(hoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putData("SysID/Hood", new SparkSysIDTest(hoodMotor, this, 2,0.1,0.9, encoder::getPosition));
    SmartDashboard.putData("Subsystem/TurretHood",this);

    //SmartDashboard.putData("Subsystem/TurretHood/HoodPos", goTo);
    //SmartDashboard.putData("Subsystem/TurretHood/HoodPosSet", new InstantCommand(()->hoodTo(goTo)));
  }

  public void setUpMap()
  {
    // map.put(0.0,0.0);
  }

  @Override
  public void periodic() 
  {

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
