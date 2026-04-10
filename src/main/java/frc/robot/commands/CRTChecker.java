// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.yams.units.EasyCRT;
import frc.lib.yams.units.EasyCRTConfig;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turdexer;
import frc.robot.subsystems.TurretRotate;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CRTChecker extends Command {
  public EasyCRT crt;
  public EasyCRTConfig crtConfig;

  public TurretRotate rotate;
  public Turdexer turdexer;
  

  public AbsoluteEncoder encoder1;
  public AbsoluteEncoder encoder2;
  /** Creates a new CRTChecker. */
  public CRTChecker(RobotContainer robotContainer) 
  {
    
    rotate = robotContainer.rotate;
    turdexer = robotContainer.turdexer;

    addRequirements(rotate); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    encoder1 = turdexer.lateralTurdexerMotor.getAbsoluteEncoder();
    encoder2 = turdexer.medialTurdexerMotor.getAbsoluteEncoder();

    crtConfig = new EasyCRTConfig(()->Rotations.of(encoder1.getPosition()), ()->Rotations.of(encoder2.getPosition()))
        .withCommonDriveGear(
            /* commonRatio (mech:drive) */ 1.0,
            /* driveGearTeeth */ 84,
            /* encoder1Pinion */ 10,
            /* encoder2Pinion */ 11)
        .withAbsoluteEncoderOffsets(Rotations.of(0.450728), Rotations.of(0.0018493935)) // set after mechanical zero
        .withMechanismRange(Rotations.of(0.0), Rotations.of(0.75)) // 0 deg to +270 deg
        .withMatchTolerance(Rotations.of(0.5)) // ~1.08 deg at encoder2 for the example ratio
        .withAbsoluteEncoderInversions(true, true);

    crt = new EasyCRT(crtConfig);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    crt.getAngleOptional().ifPresentOrElse(
      angle -> {SmartDashboard.putNumber("Commands/CRT/Angle", angle.in(Degrees));}, 
      () -> {SmartDashboard.putNumber("Commands/CRT/Angle", -1.0);});

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
