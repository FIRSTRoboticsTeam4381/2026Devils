// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
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
  

  public TurretShoot() 
  {
    shootMotor1 = new SparkFlex(CanIDs.SHOOT_MOTOR_1_ID, MotorType.kBrushless);
    shootMotor2 = new SparkFlex(CanIDs.SHOOT_MOTOR_2_ID, MotorType.kBrushless);

    SparkFlexConfig shootMotor1Config = new SparkFlexConfig();
      shootMotor1Config
        .smartCurrentLimit(40)
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
  }

  @Override
  public void periodic() 
  {

  }

  public Command shootOn()
  {
    return new InstantCommand(() -> shootMotor1.set(.65),this).repeatedly().withName("ShooterOn");
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
