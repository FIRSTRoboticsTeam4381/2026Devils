package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.CanIDs;
import frc.robot.Constants;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

@Logged
public class SwerveModule {
    public int moduleNumber;
    private SparkMax mAngleMotor;
    private SparkFlex mDriveMotor;

    private SparkAbsoluteEncoder absoluteEncoder;

    private RelativeEncoder distanceEncoder;

    private double desiredAngle;
    private double lastSpeed;

    private static final SparkFlexConfig DRIVE_CONFIG = new SparkFlexConfig(){{
        
            closedLoopRampRate(0.0);
            openLoopRampRate(0.025);
            smartCurrentLimit(80);
            idleMode(IdleMode.kBrake);
            inverted(true);    
            
        closedLoop
            .p( 3.1534E-05)
            .i(0.0)
            .d(0.0);

        closedLoop.feedForward.kV(0.0);

        encoder
            .positionConversionFactor(Constants.Swerve.WHEEL_CIRCUMFERENCE / Constants.Swerve.driveGearRatio)
            .velocityConversionFactor(Constants.Swerve.WHEEL_CIRCUMFERENCE / Constants.Swerve.driveGearRatio / 60.0);
            
    }};


    private static final SparkMaxConfig ANGLE_CONFIG = new SparkMaxConfig(){{
        smartCurrentLimit(30);
        idleMode(IdleMode.kBrake);
        inverted(true);    
        
    
        closedLoop
            .p(0.06)
            .i(0.0)
            .d(0.25)
            .feedForward.kV(0.0);
        closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 360);

        absoluteEncoder.positionConversionFactor(360);
        absoluteEncoder.inverted(true);
    }};
            

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.018231, 1.9676, 0.68184);

    public SwerveModule(int moduleNumber, CanIDs.SwerveModuleCanIDs moduleConstants){
        this.moduleNumber = moduleNumber;

        /* Angle Motor Config */
        mAngleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        
        
        mAngleMotor.configure(ANGLE_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        /* Drive Motor Config */
        mDriveMotor = new SparkFlex(moduleConstants.driveMotorID, MotorType.kBrushless);
        
        
        mDriveMotor.configure(DRIVE_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        

        /* Angle Encoder Config */
        absoluteEncoder = mAngleMotor.getAbsoluteEncoder();
        
        distanceEncoder = mDriveMotor.getEncoder();
        
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop)
    {
        // Allow wheel to flip to reverse if that angle is closer
        desiredState.optimize(getState().angle);

        // Slow down drive wheel if it is off-target
        // This should save on energy, tread wear, and odometry accuracy
        desiredState.speedMetersPerSecond *= desiredState.angle.minus(getAngle()).getCos();

         if(isOpenLoop){ // TELEOP 
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED; 
            mDriveMotor.set(percentOutput * -1);
        } 
        else{ // AUTO 
            double velocity = desiredState.speedMetersPerSecond / Constants.Swerve.WHEEL_CIRCUMFERENCE / Constants.Swerve.driveGearRatio * 60;
            mDriveMotor.getClosedLoopController().setSetpoint(velocity * -1, 
                ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward.calculate(desiredState.speedMetersPerSecond * -1));
        } 

        double angle = desiredState.angle.getDegrees();
        mAngleMotor.getClosedLoopController().setSetpoint(angle, ControlType.kPosition);
        desiredAngle = angle;
    }
    
    
    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(absoluteEncoder.getPosition());
    }

    public double getDesiredAngle(){
        return desiredAngle;
    }

    public double getDesiredSpeed(){
        return lastSpeed;
    }

    public SwerveModuleState getState(){
        double velocity = distanceEncoder.getVelocity() * driveInvert(); //Units configured to m/s
        Rotation2d angle = getAngle();
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition(){
        double distance = distanceEncoder.getPosition() * driveInvert(); //Units configured to m
        Rotation2d angle = getAngle();
        return new SwerveModulePosition(distance, angle);
    }


    // Stuff for SysID drivetrain tests
    public void setVoltage(double v)
    {
        mAngleMotor.getClosedLoopController().setSetpoint(0, ControlType.kPosition);
        mDriveMotor.setVoltage(v);
    }

    public void logSysIDData(SysIdRoutineLog log)
    {
        log.motor("m"+moduleNumber).voltage(
            edu.wpi.first.units.Units.Volts.of(mDriveMotor.getAppliedOutput() * RobotController.getBatteryVoltage())
            ).linearVelocity(edu.wpi.first.units.Units.MetersPerSecond.of(mDriveMotor.getEncoder().getVelocity()))
            .linearPosition(edu.wpi.first.units.Units.Meters.of(mDriveMotor.getEncoder().getPosition()));
    }

    public int driveInvert()
    {
        if(Constants.Swerve.driveMotorInvert)
            return -1;
        else
            return 1;
    }

    
}
