// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TurretHood;
import frc.robot.subsystems.TurretRotate;
import frc.robot.subsystems.TurretShoot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LockOn extends Command {
  
  public Swerve swerve;
  public TurretHood hood;
  public TurretRotate rotate;
  public TurretShoot shoot;

  public Pose2d current;
  public Pose2d hub;
  public Pose2d left;
  public Pose2d right;
  public Pose2d target;
  public Pose2d theoTarget;

  public double velX;
  public double velY;

  public double ballVel;

  public double timeOfFlight;

  public double dist;

  public InterpolatingDoubleTreeMap shootMap;

  public Optional<Alliance> allaince;
  public Translation2d vector;
  public Translation2d theoVector;

  public ChassisSpeeds chassisSpeeds;

  public Field2d field;

  public LockOn(RobotContainer robotContainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerve = robotContainer.swerve;
    hood = robotContainer.hood;
    rotate = robotContainer.rotate;
    shoot = robotContainer.shoot;
    addRequirements(hood,rotate,shoot); // add shoot back later when ready
  }

  
  @Override
  public void initialize() 
  {
    field = new Field2d();
    shootMap = new InterpolatingDoubleTreeMap();

    allaince = DriverStation.getAlliance();
    current = swerve.getPose(); // Make it factor in turret offset
    
    if(allaince.get() == Alliance.Blue) 
    {
      hub = new Pose2d(4.621,4.041,new Rotation2d());
      right = new Pose2d(2,6.0,new Rotation2d());
      left = new Pose2d(2,2.0,new Rotation2d());
    }
    if(allaince.get() == Alliance.Red) 
    {
      hub = new Pose2d(11.919,4.041,new Rotation2d());
      left = new Pose2d(14,6.0,new Rotation2d());
      right = new Pose2d(14,2.0,new Rotation2d());
    }


    // Maps

    shootMap.put(2.1,3200.0);
    shootMap.put(3.16,3723.0);
    shootMap.put(3.4,3700.0);
    shootMap.put(4.8,5100.0);
    shootMap.put(5.5,5800.0);
    shootMap.put(8.0,6000.0);
  }

  
  @Override
  public void execute() 
  { 
    chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getRobotRelativeSpeeds(),swerve.swerveOdometry.getEstimatedPosition().getRotation());
    current = swerve.getPose();
    velX = chassisSpeeds.vxMetersPerSecond;
    velY = chassisSpeeds.vyMetersPerSecond;

    if(allaince.get() == Alliance.Red)
    {
      // If in the red zone shoot at hub otherwise shoot left or right of it from the neutral zone
      if(current.getX()>12.5)
      {target = hub;}
      else if(current.getX()<12.5 && current.getY()<4)
      {target = right;}
      else
      {target = left;}
    }

    if(allaince.get() == Alliance.Blue)
    {
      // If in the blue zone shoot at hub otherwise shoot left or right of it from the neutral zone
      if(current.getX()<4)
      {target = hub;}
      else if(current.getX()>4 && current.getY()>4)
      {target = right;}
      else
      {target = left;}
    }


    // Adjust then give numbers

    vector = current.minus(target).getTranslation();
    
    
    dist = vector.getNorm();

    ballVel = (2*Math.PI*(0.0381)*shootMap.get(dist))/60;

    timeOfFlight = 3*dist/ballVel;
    
    theoTarget = new Pose2d(target.getX()-(velX*timeOfFlight), target.getY()-(velY*timeOfFlight), new Rotation2d());
    theoVector = current.minus(theoTarget).getTranslation();

    rotate.point(current.getRotation().minus(theoVector.getAngle()).getDegrees());

    shoot.speedFromDist(theoVector.getNorm());
    hood.angleFromDist(theoVector.getNorm());

    field.setRobotPose(theoTarget);

    SmartDashboard.putNumber("Commands/LockOn/DistFromPoint", vector.getNorm());
    SmartDashboard.putData("Commands/LockOn/TheoTargetField",field);
    SmartDashboard.putNumber("Commands/LockOn/BallVel", ballVel);
    SmartDashboard.putNumber("Commands/LockOn/TimeOfFlight", timeOfFlight);
    
  }

  
  @Override
  public void end(boolean interrupted) 
  {
    rotate.point(0);
    shoot.shootOn(0);
    hood.angleFromDist(0);
  }

  @Override
  public boolean isFinished() {
    
    return false;
  }
}
