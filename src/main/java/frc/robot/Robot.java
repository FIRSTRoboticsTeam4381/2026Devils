// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.urcl.URCL;

import com.revrobotics.util.StatusLogger;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.logging.RIOAlerts;
import frc.lib.logging.RadioLogger;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  @SuppressWarnings("unused")
  private RadioLogger robotRadioLogger, apRadioLogger;

  public Robot() {

    // Forward ports for pis
    PortForwarder.add(5801, "pi1.local", 5800);
    PortForwarder.add(5802, "pi2.local", 5800);

    for (int port = 1180; port <= 1190; port++){
      PortForwarder.add(port, "pi1.local", port);
    }

    // We are using URCL instead of Rev's version
    StatusLogger.disableAutoLogging();

    // Start logging system
    DataLogManager.logConsoleOutput(true);
    DataLogManager.start();

    // Start logging REV can network traffic
    URCL.start(CanIDs.URCL_IDS);

    // Start logging driver station data
    DriverStation.startDataLog(DataLogManager.getLog());
    
    // Start epilogue logging
    Epilogue.bind(this);

    m_robotContainer = new RobotContainer();

    // Radio logging setup
    robotRadioLogger = new RadioLogger("http://10.43.81.1/status");
    apRadioLogger = new RadioLogger("http://10.43.81.4/status");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    RIOAlerts.logRioData();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    // Check if joysticks aren't zeroed
    // This should be ambiguous about what type of controller is plugged in
    RIOAlerts.checkJoysticZeroing();
  }

  

  @Override
  public void disabledExit() {
    RIOAlerts.clearJoystickZeroAlerts();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  
}
