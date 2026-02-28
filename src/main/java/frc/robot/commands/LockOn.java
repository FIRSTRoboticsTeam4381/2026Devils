// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
  public Optional<Alliance> allaince;
  public Translation2d vector;

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
    allaince = DriverStation.getAlliance();
    current = swerve.getPose();
    
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
  }

  
  @Override
  public void execute() 
  {
    current = swerve.getPose();
    

    

    if(allaince.get() == Alliance.Red)
    {
      // If in the red zone shoot at hub otherwise shoot left or right of it from the neutral zone
      if(current.getX()>12.5)
      {
        vector = current.minus(hub).getTranslation();
        rotate.point(current.getRotation().minus(vector.getAngle()).getDegrees());
      }
      else if(current.getX()<12.5 && current.getY()<4)
      {
        vector = current.minus(right).getTranslation();
        rotate.point(current.getRotation().minus(vector.getAngle()).getDegrees());
      }
      else
      {
        vector = current.minus(left).getTranslation();
        rotate.point(current.getRotation().minus(vector.getAngle()).getDegrees());
      }
      shoot.speedFromDist(vector.getNorm());
      hood.angleFromDist(vector.getNorm());
    }

    if(allaince.get() == Alliance.Blue)
    {
      // If in the blue zone shoot at hub otherwise shoot left or right of it from the neutral zone
      if(current.getX()<4)
      {
        vector = current.minus(hub).getTranslation();
        rotate.point(current.getRotation().minus(vector.getAngle()).getDegrees());
      }
      else if(current.getX()>4 && current.getY()>4)
      {
        vector = current.minus(right).getTranslation();
        rotate.point(current.getRotation().minus(vector.getAngle()).getDegrees());
      }
      else
      {
        vector = current.minus(left).getTranslation();
        rotate.point(current.getRotation().minus(vector.getAngle()).getDegrees());
      }
      shoot.speedFromDist(vector.getNorm());
      hood.angleFromDist(vector.getNorm());
    }

    SmartDashboard.putNumber("Commands/LockOn/DistFromPoint", vector.getNorm());

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
