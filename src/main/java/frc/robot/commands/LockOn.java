// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turdexer;
import frc.robot.subsystems.TurretHood;
import frc.robot.subsystems.TurretRotate;
import frc.robot.subsystems.TurretShoot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LockOn extends Command 
{
  
  public Swerve swerve;
  public TurretHood hood;
  public TurretRotate rotate;
  public TurretShoot shoot;
  public Turdexer turdexer;

  public Pose2d current;
  public Pose2d hub;
  public Pose2d left;
  public Pose2d right;
  public Pose2d target;
  public Pose2d theoTarget;

  public double velX;
  public double velY;

  public double timeOfFlight;

  public double dist;

  public InterpolatingDoubleTreeMap timeOfFlightMap;

  public Optional<Alliance> allaince;
  public Translation2d vector;
  public Translation2d theoVector;

  public ChassisSpeeds chassisSpeeds;

  public Field2d field;

  public LockOn(RobotContainer robotContainer) 
  {
    swerve = robotContainer.swerve;
    hood = robotContainer.hood;
    rotate = robotContainer.rotate;
    shoot = robotContainer.shoot;
    turdexer = robotContainer.turdexer;
    addRequirements(hood,rotate,shoot); 
  }

  
  @Override
  public void initialize() 
  {
    field = new Field2d();
    timeOfFlightMap = new InterpolatingDoubleTreeMap();

    //////////////////////////////////////////////////////
    timeOfFlightMap.put(2.548, 0.95);
    timeOfFlightMap.put(2.126, 0.84);
    timeOfFlightMap.put(3.422, 1.05);
    timeOfFlightMap.put(3.092, 1.06);
    timeOfFlightMap.put(3.732, 1.00);
    timeOfFlightMap.put(3.065, 1.15);
    timeOfFlightMap.put(3.065, 1.15);


    //////////////////////////////////////////////////////

    // VIDEO TIMES:
    // 1.45 2.40
    // 11.08 11.92
    // 5.45 6.50
    // 3.37 4.43
    // 4.50 5.50

    allaince = DriverStation.getAlliance();

    current = swerve.getPose().plus(new Transform2d(Units.inchesToMeters(-7),Units.inchesToMeters(6),new Rotation2d()));

    
    if(allaince.get() == Alliance.Blue) 
    {
      hub = new Pose2d(4.75,4.041,new Rotation2d());
      right = new Pose2d(0.6,7.4,new Rotation2d());
      left = new Pose2d(0.6,0.6,new Rotation2d());
    }
    if(allaince.get() == Alliance.Red) 
    {
      hub = new Pose2d(11.5,4.041,new Rotation2d());
      left = new Pose2d(15.8,7.4,new Rotation2d());
      right = new Pose2d(15.8,0.6,new Rotation2d());
    } 
  }
  
  @Override
  public void execute() 
  { 
    chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getRobotRelativeSpeeds(),swerve.swerveOdometry.getEstimatedPosition().getRotation());
    
    velX = chassisSpeeds.vxMetersPerSecond;
    velY = chassisSpeeds.vyMetersPerSecond;

    current = swerve.getPose().plus(new Transform2d(Units.inchesToMeters(-8),0,new Rotation2d()));

    if(allaince.get() == Alliance.Red)
    {
      // If in the red zone shoot at hub otherwise shoot left or right of it from the neutral zone
      if(current.getX()>11.25)
      {target = hub;}
      else if(current.getX()<12.5 && current.getY()<4)
      {target = right;}
      else
      {target = left;}
    }

    if(allaince.get() == Alliance.Blue)
    {      // If in the blue zone shoot at hub otherwise shoot left or right of it from the neutral zone
      if(current.getX()<4.75)
      {target = hub;}
      else if(current.getX()>4 && current.getY()>4)
      {target = right;}
      else
      {target = left;}
    }


    // Adjust then give numbers
    vector = current.minus(target).getTranslation();
    
    dist = vector.getNorm();

    timeOfFlight = timeOfFlightMap.get(dist);
    
    theoTarget = new Pose2d(target.getX()-(velX*timeOfFlight), target.getY()-(velY*timeOfFlight), new Rotation2d());
    theoVector = current.minus(theoTarget).getTranslation();

    rotate.point(-current.getRotation().minus(theoVector.getAngle()).getDegrees() + 143.5); // Straight is 143 deg

    shoot.speedFromDist(theoVector.getNorm());
    hood.angleFromDist(theoVector.getNorm());

    field.setRobotPose(theoTarget);
    field.getObject("turretPos").setPose(current);


    SmartDashboard.putNumber("Commands/LockOn/DistFromPoint", vector.getNorm());
    SmartDashboard.putData("Commands/LockOn/TheoTargetField",field);
    SmartDashboard.putNumber("Commands/LockOn/TimeOfFlight", timeOfFlight);
  }

  
  @Override
  public void end(boolean interrupted) 
  {
    rotate.point(143);
    shoot.shootOn(0);
    hood.angleFromDist(0);
    turdexer.turdexerOff();
  }

  @Override
  public boolean isFinished() 
  {  
    return false;
  }
}
