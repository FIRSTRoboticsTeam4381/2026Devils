// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.commands;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.TurretShoot;

/**
 * Utility class that creates a Sequential Command containing the four SysID tests
 * for simple single-DOF mechanisms.
 */
public class SparkSysIDTest extends SequentialCommandGroup {
  
  
  /**
   * Constructs a sequential command containing the four SysID tests, separated by a delay.
   * 
   * The first test is a Forward test, so the mechamism should be positioned at its lowest
   * point (as seen by the encoder, and in the negative direction of travel.)
   * 
   * This constructor won't stop the test based on encoder positions! This is meant for things
   * like shooter wheels that don't have a physical limit. If the mechanism needs to stop safely
   * before an end point, use the other constructors.
   * 
   * @param motor The motor driving this mechanism. If the mechanism has multiple motors,
   * this should be the "leader" motor.
   * @param s The subsystem for this command to require
   * @param delayBetweenTests How long to wait between tests. This should be long enough for
   * the mechanism to come to a stop, but in the case of an arm or elevator, not long enough
   * for it to fall back down.
   */
  public SparkSysIDTest(SparkBase motor, Subsystem s, double delayBetweenTests) {
      this(motor, s, delayBetweenTests, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, () -> 0.0, new SysIdRoutine.Config());
    }

  /**
   * Constructs a sequential command containing the four SysID tests, separated by a delay.
   * 
   * The first test is a Forward test, so the mechamism should be positioned at its lowest
   * point (as seen by the encoder, and in the negative direction of travel.)
   * 
   * This constructor won't stop the test based on encoder positions! This is meant for things
   * like shooter wheels that don't have a physical limit. If the mechanism needs to stop safely
   * before an end point, use the other constructors.
   * 
   * @param motor The motor driving this mechanism. If the mechanism has multiple motors,
   * this should be the "leader" motor.
   * @param s The subsystem for this command to require
   * @param delayBetweenTests How long to wait between tests. This should be long enough for
   * the mechanism to come to a stop, but in the case of an arm or elevator, not long enough
   * for it to fall back down.
   * @param config Overrides the default SysID test configuration.
   */
  public SparkSysIDTest(SparkBase motor, Subsystem s, double delayBetweenTests, SysIdRoutine.Config config) {
      this(motor, s, delayBetweenTests, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, () -> 0.0, config);
    }

  /**
   * Constructs a sequential command containing the four SysID tests, separated by a delay.
   * 
   * The first test is a Forward test, so the mechamism should be positioned at its lowest
   * point (as seen by the encoder, and in the negative direction of travel.)
   * 
   * This constructor will cut off tests when the supplier for encoder position reaches lowerStop
   * or upperStop, depending on the test. Make sure these are reached slightly *before* the 
   * mechanism hits a hard stop, limit switch, or soft encoder limit! Hitting another limit first 
   * will cause the stop from that limit to appear in the test data, skewing it. 
   * 
   * If this mechanism doesn't need to stop at encoder limits, use a constructor
   * without the lowerStop/upperStop/readEncoder parameters.
   * 
   * @param motor The motor driving this mechanism. If the mechanism has multiple motors,
   * this should be the "leader" motor.
   * @param s The subsystem for this command to require
   * @param delayBetweenTests How long to wait between tests. This should be long enough for
   * the mechanism to come to a stop, but in the case of an arm or elevator, not long enough
   * for it to fall back down.
   * @param lowerStop When moving in reverse, the tests will stop when they pass this value.
   * Make this slightly higher than the mechanisms "real" stopping point.
   * @param upperStop When moving forwards, the tests will stop when they pass this value.
   * Make this slightly lower than the mechanisms "real" stopping point.
   * @param readEncoder Put a supplier here that reads the position of the encoder
   * being used for lowerStop and upperStop.
   */
  public SparkSysIDTest(SparkBase motor, Subsystem s, double delayBetweenTests,
    double lowerStop, double upperStop, Supplier<Double> readEncoder) {
      this(motor, s, delayBetweenTests, lowerStop, upperStop, readEncoder, new SysIdRoutine.Config());
    }
  
    /**
   * Constructs a sequential command containing the four SysID tests, separated by a delay.
   * 
   * The first test is a Forward test, so the mechamism should be positioned at its lowest
   * point (as seen by the encoder, and in the negative direction of travel.)
   * 
   * Tests will cut off when the supplier for encoder position reaches lowerStop or upperStop,
   * depending on the test. Make sure these are reached slightly *before* the mechanism hits
   * a hard stop, limit switch, or soft encoder limit! Hitting another limit first will cause
   * the stop from that limit to appear in the test data, skewing it. 
   * 
   * @param motor The motor driving this mechanism. If the mechanism has multiple motors,
   * this should be the "leader" motor.
   * @param s The subsystem for this command to require
   * @param delayBetweenTests How long to wait between tests. This should be long enough for
   * the mechanism to come to a stop, but in the case of an arm or elevator, not long enough
   * for it to fall back down.
   * @param lowerStop When moving in reverse, the tests will stop when they pass this value.
   * Make this slightly higher than the mechanisms "real" stopping point.
   * @param upperStop When moving forwards, the tests will stop when they pass this value.
   * Make this slightly lower than the mechanisms "real" stopping point.
   * @param readEncoder Put a supplier here that reads the position of the encoder
   * being used for lowerStop and upperStop.
   * @param config Overrides the default SysID test configuration.
   */
  public SparkSysIDTest(SparkBase motor, Subsystem s, double delayBetweenTests,
    double lowerStop, double upperStop, Supplier<Double> readEncoder, SysIdRoutine.Config config) {

    SysIdRoutine routine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                    (v) -> {
                      motor.setVoltage(v.in(edu.wpi.first.units.Units.Volts));  
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

    public SparkSysIDTest(SparkFlexConfig shootMotor1Config, TurretShoot s, int delayBetweenTests) {
        //TODO Auto-generated constructor stub
    }
}
