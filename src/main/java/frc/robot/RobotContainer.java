// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.commands.TeleopSwerve;
import frc.lib.subsystems.PhotonCam;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Swerve;

@Logged
public class RobotContainer {
  
  // Controllers
  public final CommandXboxController driver = new CommandXboxController(0);
  public final CommandXboxController specialist = new CommandXboxController(1);
  
  // If you are using a button board, uncomment these and comment out specialist above
  // You may also want to adjust the un-zero'd joystick check in lib/controls/JoystickUtils.java
  //public CommandGenericHID buttonBoard1 = new CommandGenericHID(1);
  //public CommandGenericHID buttonBoard2 = new CommandGenericHID(2);

  //Auto Chooser
  SendableChooser<Autos.PreviewAuto> autoChooser = new SendableChooser<>();

  // Subsystems
  public final Swerve swerve = new Swerve();

  // TODO set camera names, coordinates, and angles relative to the robot's center
  //public final PhotonCam camA = new PhotonCam("Camera A", new Transform3d(new Translation3d(Units.inchesToMeters(-10.375), Units.inchesToMeters(-7.3125),  Units.inchesToMeters(8.5)), new Rotation3d(0,Math.PI/-6,Math.PI/-4-Math.PI)) );
  //public final PhotonCam camB = new PhotonCam("Camera B", new Transform3d(new Translation3d(Units.inchesToMeters(-10.375), Units.inchesToMeters(7.3125),  Units.inchesToMeters(8.5)), new Rotation3d(0,Math.PI/-6,Math.PI/4-Math.PI)) );
  //public final PhotonCam camC = new PhotonCam("Camera C", new Transform3d(new Translation3d(Units.inchesToMeters(-10.375), Units.inchesToMeters(7.3125),  Units.inchesToMeters(8.5)), new Rotation3d(0,Math.PI/-6,Math.PI/4-Math.PI)) );
  //public final PhotonCam camD = new PhotonCam("Camera D", new Transform3d(new Translation3d(Units.inchesToMeters(-10.375), Units.inchesToMeters(7.3125),  Units.inchesToMeters(8.5)), new Rotation3d(0,Math.PI/-6,Math.PI/4-Math.PI)) );


  // Constructor: set up the robot! 
  public RobotContainer() {
    robotReference = this;

    // Set default commands here




    // Set up autonomous picker
    // Add any autos you want to be able to select below
    autoChooser.setDefaultOption("None", Autos.none());
    autoChooser.addOption("Test", Autos.testAuto());
    

    // Add auto controls to the dashboard
    SmartDashboard.putData("Choose Auto:", autoChooser);
    SmartDashboard.putData(CommandScheduler.getInstance());
    autoChooser.onChange((listener) -> {
      if(listener!=null)
        listener.showPreview();
      });
    SmartDashboard.putNumber("Start Delay",0);

    
    // Configure button bindings
    configureBindings();
  }

  private void configureBindings() {
    // Button to re-zero swerve drive
    driver.back()
      .onTrue(new InstantCommand(() -> swerve.zeroYaw()).ignoringDisable(true));
    
    // Default teleop swerve command
    swerve.setDefaultCommand(new TeleopSwerve(swerve, 
            driver::getLeftY,
            driver::getLeftX,
            driver::getRightX));

    // Example "slow mode" with 60% modififier on right bumper
    driver.rightBumper().whileTrue(
        new TeleopSwerve(swerve, 
            driver::getLeftY,
            driver::getLeftX,
            driver::getRightX,
            0.6));

    // Set the wheels to an "X" shape to make the robot more difficult to push
    driver.x().whileTrue(swerve.brake());

    // Button to revert to robot-oriented drive in an emergency
    driver.start().toggleOnTrue(
      new TeleopSwerve(swerve, 
            driver::getLeftY,
            driver::getLeftX,
            driver::getRightX,
            1.0,
            true,
            false,
            false));

    // Button to cancel running actions
    specialist.back().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
    
            
    // TODO Your Controls Here!




  }

  public Command getAutonomousCommand() {
    double startDelay=SmartDashboard.getNumber("Start Delay", 0);
    return new SequentialCommandGroup( 
      new WaitCommand(startDelay), 
      new ScheduleCommand(autoChooser.getSelected().auto)
    ); 
  }


  // Static reference to the robot class
  // Use getRobot() to get the robot object
  private static RobotContainer robotReference;

  /**
   * Get a reference to the RobotContainer object in use
   * @return the active RobotContainer object
   */
  public static RobotContainer getRobot()
  {
    return robotReference;
  }

  

  
}
