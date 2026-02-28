// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;
import java.util.concurrent.locks.Lock;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.commands.TeleopSwerve;
import frc.lib.subsystems.PhotonCam;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turdexer;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TurretShoot;
import frc.robot.subsystems.TurretHood;
import frc.robot.subsystems.TurretRotate;
import frc.robot.commands.LockOn;

@Logged
public class RobotContainer {
  
  // Controllers
  public final CommandXboxController driver = new CommandXboxController(0);
  public final CommandXboxController specialist = new CommandXboxController(1);
  
  public CommandGenericHID buttonBoard1 = new CommandGenericHID(2);
  public CommandGenericHID buttonBoard2 = new CommandGenericHID(3);
  // If you are using a button board, uncomment these and comment out specialist above
  // You may also want to adjust the un-zero'd joystick check in lib/controls/JoystickUtils.java

  //Auto Chooser
  SendableChooser<Autos.PreviewAuto> autoChooser = new SendableChooser<>();

  // Subsystems
  public final Swerve swerve = new Swerve();
  //public final Climb climb = new Climb();
  public final Indexer indexer = new Indexer();
  public final Intake intake = new Intake();
  public final TurretShoot shoot = new TurretShoot();
  public final TurretHood hood = new TurretHood();
  public final TurretRotate rotate = new TurretRotate();
  public final Turdexer turdexer = new Turdexer();
  

  // TODO set camera names, coordinates, and angles relative to the robot's center
  public final PhotonCam camA = new PhotonCam("Camera A", new Transform3d(new Translation3d(Units.inchesToMeters(-12.33496), Units.inchesToMeters(-3.11445),  Units.inchesToMeters(6.15733)), new Rotation3d(0,Math.toRadians(-30),Math.toRadians(-45)-Math.PI)) );
  public final PhotonCam camB = new PhotonCam("Camera B", new Transform3d(new Translation3d(Units.inchesToMeters(-12.33496), Units.inchesToMeters(3.11445),  Units.inchesToMeters(6.15733)), new Rotation3d(0,Math.toRadians(-30),Math.toRadians(45)-Math.PI)) );
  public final PhotonCam camC = new PhotonCam("Camera C", new Transform3d(new Translation3d(Units.inchesToMeters(-9.27168), Units.inchesToMeters(-12.28787),  Units.inchesToMeters(12.40401)), new Rotation3d(0,Math.toRadians(-30),Math.toRadians(-60))) );
  public final PhotonCam camD = new PhotonCam("Camera D", new Transform3d(new Translation3d(Units.inchesToMeters(-9.27168), Units.inchesToMeters(12.28787),  Units.inchesToMeters(12.40401)), new Rotation3d(0,Math.toRadians(-30),Math.toRadians(60))) );


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

    NamedCommands.registerCommand("IntakeDeploy", intake.intakeDeploy());
    NamedCommands.registerCommand("IntakeUndeploy", intake.intakeUndeploy());
    NamedCommands.registerCommand("IntakeIn", intake.intakeIn());
    NamedCommands.registerCommand("LockOn", new LockOn(this));
    NamedCommands.registerCommand("DexerThrough", indexer.indexerThrough().alongWith(turdexer.turdexerThrough()));
    NamedCommands.registerCommand("DexerOff", indexer.indexerOff().alongWith(turdexer.turdexerOff()));

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
    specialist.x().toggleOnTrue(intake.intakeDeploy());
    specialist.b().toggleOnTrue(intake.intakeUndeploy());
    specialist.a().whileTrue(intake.intakeIn());
    specialist.y().whileTrue(intake.intakeOut());
    specialist.leftBumper().toggleOnTrue(new LockOn(this));
    specialist.rightBumper().whileTrue(turdexer.turdexerThrough()).whileTrue(indexer.indexerThrough());
    specialist.rightTrigger().whileTrue(turdexer.turdexerBack()).whileTrue(indexer.indexerBack());
    
    // Button board controls
    // Board 1:  Turret:axis0 Hood:axis1 Override:button1 Fire:button2 ---- Board 2:  Shooter:axis0
    buttonBoard1.button(1)
    .whileTrue(new InstantCommand( () -> buttonBoard1.button(2).toggleOnTrue(shoot.shootOnOverride()))) // Shoot speed going to need adjustment inside TurretShoot
    .whileTrue(new InstantCommand( () -> shoot.shootOn(((buttonBoard2.getRawAxis(0) + 1) / 2 * 6000)))) // (0 to 7000) Figure out velocity (7000 #) later
    .whileTrue(new InstantCommand( () -> hood.angle((buttonBoard1.getRawAxis(1) + 1) / 2 * 0.5))) // (0 to 0.5) Figure out 0.5 # later
    .whileTrue(new InstantCommand( () -> rotate.point(buttonBoard1.getRawAxis(0) * 105))); // (-1 to 1 * 105 in degrees) Should be right range    

    // Climb later

    // Temp Manual
    //specialist.     hood.joystickCont()


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
