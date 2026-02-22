// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkBase;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.commands.SparkSysIDTest;

/** Add your docs here. */
public class GeekedSysID extends SparkSysIDTest 
{
    public GeekedSysID(SparkBase motor, Subsystem s, double delayBetweenTests,
    double lowerStop, double upperStop, Supplier<Double> readEncoder, SysIdRoutine.Config config) 
    {
        super( motor,  s,  delayBetweenTests, lowerStop,  upperStop,  readEncoder, config);
    SysIdRoutine routine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                    (v) -> {
                      motor.setVoltage(-v.in(edu.wpi.first.units.Units.Volts));  
                      /*for(int j = 0; j < mSwerveMods.length; j++)
                        {
                            mSwerveMods[j].setVoltage(v.in(edu.wpi.first.units.Units.Volts));
                        }*/
                    }, 
                    null,
                    //(log) ->
                    //{
                        /*for(int j = 0; j < mSwerveMods.length; j++)
                        {
                            mSwerveMods[j].logSysIDData(log);
                        }
                        
                        log.motor("m"+moduleNumber).voltage(
            edu.wpi.first.units.Units.Volts.of(mDriveMotor.getAppliedOutput() * RobotController.getBatteryVoltage())
            ).linearVelocity(edu.wpi.first.units.Units.MetersPerSecond.of(mDriveMotor.getEncoder().getVelocity()))
            .linearPosition(edu.wpi.first.units.Units.Meters.of(mDriveMotor.getEncoder().getPosition()));

                        
                        */

                        /*log.motor("id "+motor.getDeviceId())
                          //.voltage(edu.wpi.first.units.Units.Volts.of(motor.getAppliedOutput() * RobotController.getBatteryVoltage()))
                          .voltage(edu.wpi.first.units.Units.Volts.of(motor.getAppliedOutput() * motor.getBusVoltage()))
                          .*/
                        
                    //}, 
                    s)
            );
    
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      routine.dynamic(Direction.kForward).until(() -> readEncoder.get() >= upperStop),
      new InstantCommand(() -> motor.set(0.0)),
      new WaitCommand(delayBetweenTests),
      routine.dynamic(Direction.kReverse).until(() -> readEncoder.get() <= lowerStop),
      new InstantCommand(() -> motor.set(0.0)),
      new WaitCommand(delayBetweenTests),
      routine.quasistatic(Direction.kForward).until(() -> readEncoder.get() >= upperStop),
      new InstantCommand(() -> motor.set(0.0)),
      new WaitCommand(delayBetweenTests),
      routine.quasistatic(Direction.kReverse).until(() -> readEncoder.get() <= lowerStop),
      new InstantCommand(() -> motor.set(0.0))
    );
  }
}
