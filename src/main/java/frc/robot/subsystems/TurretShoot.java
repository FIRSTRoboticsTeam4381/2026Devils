// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.commands.SparkSysIDTest;
import frc.robot.CanIDs;
import frc.robot.Constants;

@Logged
public class TurretShoot extends SubsystemBase {

  public SparkFlex shootMotor1;
  public SparkFlex shootMotor2;
  public InterpolatingDoubleTreeMap map;

  public Double goTo;

  public TurretShoot() 
  {
    this.goTo = 0.0;
    map = new InterpolatingDoubleTreeMap();

    shootMotor1 = new SparkFlex(CanIDs.SHOOT_MOTOR_1_ID, MotorType.kBrushless);
    shootMotor2 = new SparkFlex(CanIDs.SHOOT_MOTOR_2_ID, MotorType.kBrushless);

    setUpMap();
    SparkFlexConfig shootMotor1Config = new SparkFlexConfig();
      shootMotor1Config
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kCoast)
        .inverted(true);
        
      shootMotor1Config.closedLoop
        .p(2.2307E-06)
        .i(0)
        .d(0)
        .feedForward
        .sva(0.038845, 0.0018515, 0.0005394);

      shootMotor1Config.signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .isAtSetpointAlwaysOn(true)
        .maxMotionSetpointPositionAlwaysOn(true)
        .maxMotionSetpointVelocityAlwaysOn(true)
        .setSetpointAlwaysOn(true);
    
    SparkFlexConfig shootMotor2Config = new SparkFlexConfig();
      shootMotor2Config
        .apply(shootMotor1Config)
        .follow(shootMotor1,true)
        ;

    shootMotor1.configure(shootMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shootMotor2.configure(shootMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    setDefaultCommand(shootOff()); 

    SmartDashboard.putData("SysID/Shooter", new SparkSysIDTest(shootMotor1, this, 10));

    SmartDashboard.putData("Subsystem/TurretShoot",this);

    SmartDashboard.putNumber("Subsystem/TurretShoot/ShootSpeed", 0.0);
    SmartDashboard.putData("Subsystem/TurretShoot/ShootSpeedSet", new InstantCommand(()->shootOn(SmartDashboard.getNumber("Subsystem/TurretShoot/ShootSpeed", 0.0)),this).repeatedly());
  }

  public void setUpMap()
  {
    map.put(2.37, 3245.0);
    map.put(2.84, 3350.0);
    map.put(3.76, 3970.0);
    map.put(4.94,4850.0);
    map.put(7.49,5400.0);
    map.put(7.79,6000.0);
    
  }

  @Override
  public void periodic() 
  {
    SmartDashboard.putNumber("Subsystem/TurretShoot/Velocity", shootMotor1.getEncoder().getVelocity());
    
  }

  public void speedFromDist(double dist)
  {
    shootOn(map.get(dist));
  }

  public void shootOn(double vel)
  {
    shootMotor1.getClosedLoopController().setSetpoint(vel, ControlType.kVelocity);
  }

  public Command rev()
  {
    return new InstantCommand(() -> shootMotor1.getClosedLoopController().setSetpoint(5000, ControlType.kVelocity));
  }

  public Command shootOnOverride() {
    return new InstantCommand(() -> shootMotor1.set(0.5),this).repeatedly().withName("shooterOn OVERRIDE");
  }

  public Command shootOff()
  {
    return new InstantCommand(() -> shootMotor1.set(0),this).repeatedly().withName("ShooterOff");
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
