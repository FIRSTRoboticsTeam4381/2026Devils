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
import frc.robot.commands.CRTChecker;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turdexer;
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
  public final PhotonCam LL = new PhotonCam("LeftLeft", new Transform3d(new Translation3d(Units.inchesToMeters(-10.3369), Units.inchesToMeters(12.63032),  Units.inchesToMeters(7.63495)), new Rotation3d(180-45,Math.toRadians(-30),Math.toRadians(-105)-Math.PI)) );
  public final PhotonCam LB = new PhotonCam("LeftBack", new Transform3d(new Translation3d(Units.inchesToMeters(-13.64295), Units.inchesToMeters(11.1827),  Units.inchesToMeters(7.14753)), new Rotation3d(0,Math.toRadians(-25),Math.toRadians(20)-Math.PI)) );
  public final PhotonCam RR = new PhotonCam("RightRight", new Transform3d(new Translation3d(Units.inchesToMeters(-10.3369), Units.inchesToMeters(-12.63032),  Units.inchesToMeters(8.13495)), new Rotation3d(180-45,Math.toRadians(-30),Math.toRadians(105)-Math.PI)) );
  public final PhotonCam RB = new PhotonCam("RightBack", new Transform3d(new Translation3d(Units.inchesToMeters(-13.64295), Units.inchesToMeters(-11.1827),  Units.inchesToMeters(7.61044)), new Rotation3d(0,Math.toRadians(-25),Math.toRadians(-20)-Math.PI)) );


  // Constructor: set up the robot! 
  public RobotContainer() {
    robotReference = this;

    // Set default commands here

    

    // Named Commands
    NamedCommands.registerCommand("IntakeDeploy", intake.intakeDeploy());
    NamedCommands.registerCommand("IntakeUndeploy", intake.intakeUndeploy());
    NamedCommands.registerCommand("IntakeIn", intake.intakeIn());
    NamedCommands.registerCommand("LockOn", new LockOn(this));
    NamedCommands.registerCommand("DexerThrough", indexer.indexerThrough().alongWith(turdexer.turdexerThrough()));
    NamedCommands.registerCommand("DexerBack", indexer.indexerBack().alongWith(turdexer.turdexerBack()));
    NamedCommands.registerCommand("DexerOff", indexer.indexerOff().alongWith(turdexer.turdexerOff()));
    NamedCommands.registerCommand("Rev", shoot.rev());


    // Set up autonomous picker
    // Add any autos you want to be able to select below
    autoChooser.setDefaultOption("None", Autos.none());
    autoChooser.addOption("Test", Autos.testAuto());
    autoChooser.addOption("SimpleRightAuto - NK", Autos.SimpleRightAuto());
    autoChooser.addOption("SimpleLeftAuto - NK", Autos.SimpleLeftAuto());
    autoChooser.addOption("LeftAuto - NK", Autos.LeftAuto());
    autoChooser.addOption("MiddleAuto - JL", Autos.MiddleAuto());
    autoChooser.addOption("MoreMiddleAuto - JL", Autos.MoreMiddleAuto());
    autoChooser.addOption("RightOP - NK", Autos.RightOPAuto());
    autoChooser.addOption("LeftOSOS - NK", Autos.LeftOSOSAuto());
    

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
    specialist.x().toggleOnTrue(intake.intakeDeploy());
    specialist.b().toggleOnTrue(intake.intakeUndeploy());
    specialist.a().whileTrue(intake.intakeIn());
    specialist.y().whileTrue(intake.intakeOut());
    specialist.leftBumper().toggleOnTrue(new LockOn(this)).toggleOnTrue(turdexer.turdexerThrough());
    specialist.rightBumper().whileTrue(indexer.indexerThrough()).and(()->shoot.shootMotor1.getEncoder().getVelocity() + shoot.shootMotor2.getEncoder().getVelocity()>500).whileTrue(intake.intakeIn());
    specialist.leftTrigger().toggleOnTrue(new CRTChecker(this));     
    specialist.rightTrigger().whileTrue(indexer.indexerBack());
    
    // Button board controls
    // Board 1:  Turret:axis0 Hood:axis1 Override:button1 Fire:button2 ---- Board 2:  Shooter:axis0

    //buttonBoard1.button(1).whileTrue(shoot.shootOnOverride().repeatedly());
    buttonBoard1.button(2).and(buttonBoard1.button(1)).whileTrue(turdexer.turdexerThrough()).whileTrue(indexer.indexerThrough()).whileTrue(intake.intakeIn()); // Shoot speed going to need adjustment inside TurretShoot
    buttonBoard1.button(1) 
    .whileTrue(new InstantCommand( () -> shoot.shootOn(((buttonBoard2.getRawAxis(0) + 1) / 2 * 6000)), shoot).repeatedly()) // (0 to 7000) Figure out velocity (7000 #) later
    .whileTrue(new InstantCommand( () -> hood.angle((buttonBoard1.getRawAxis(1) + 1) / 2 * 0.95), hood).repeatedly()) // (0 to 0.5) Figure out 0.5 # later
    .whileTrue(new InstantCommand( () -> rotate.point(buttonBoard1.getRawAxis(0) * 105), rotate).repeatedly()); // (-1 to 1 * 105 in degrees) Should be right range    

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
