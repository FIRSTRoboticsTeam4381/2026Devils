package frc.robot.subsystems;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.commands.AutoCorrection;
import frc.robot.CanIDs;
import frc.robot.Constants;

@Logged
public class Swerve extends SubsystemBase{
    public SwerveDrivePoseEstimator swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;

    // Secondary references for automated logging
    private SwerveModule FL;
    private SwerveModule FR;
    private SwerveModule BL;
    private SwerveModule BR;

    StructArrayPublisher<SwerveModuleState> modStatusPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Swerve/ModuleStatus", SwerveModuleState.struct).publish();
    StructArrayPublisher<SwerveModuleState> modTargetPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Swerve/ModuleTarget", SwerveModuleState.struct).publish();

    StructPublisher<ChassisSpeeds> chasStatusPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Swerve/ChassisStatus", ChassisSpeeds.struct).publish();
    StructPublisher<ChassisSpeeds> chasTargetPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Swerve/ChassisTarget", ChassisSpeeds.struct).publish();

    public final Field2d field = new Field2d();
    private Pose2d startPose = new Pose2d(8.5, 4, Rotation2d.fromDegrees(90));


    public Swerve(){
        gyro = new AHRS(NavXComType.kUSB1);

        SmartDashboard.putData(this);

        mSwerveMods = new SwerveModule[]{
            new SwerveModule(0, CanIDs.SwerveModules.MOD0),
            new SwerveModule(1, CanIDs.SwerveModules.MOD1),
            new SwerveModule(2, CanIDs.SwerveModules.MOD2),
            new SwerveModule(3, CanIDs.SwerveModules.MOD3)
        };

        // Set references for auto logging
        FL = mSwerveMods[0];
        FR = mSwerveMods[1];
        BL = mSwerveMods[2];
        BR = mSwerveMods[3];

        swerveOdometry = new SwerveDrivePoseEstimator(Constants.Swerve.SWERVE_KINEMATICS, getGyroYaw(), getPositions(), startPose);

        try {

            // TODO check - auto
            AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                Constants.Swerve.HOLOMONIC_CONFIG,
                RobotConfig.fromGUISettings(),
                () -> {
                        Optional<Alliance> alliance = DriverStation.getAlliance();
                        if(alliance.isPresent()) {
                            return alliance.get() == Alliance.Red;
                        }
                        return false;
                    },
                this // Reference to this subsystem to set requirements
            );
        }
        catch(Exception e)
        {
            e.printStackTrace();
        }

        // Setup position logging
        SmartDashboard.putData("Field", field);
        //m_field.setRobotPose(startPose);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target").setPose(pose);

            // Tell any AutoCorrection commands where the robot is supposed to be
            AutoCorrection.target = pose;
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });


        setupSysIDTests();

        // Show a swerve module status display on Elastic
        SmartDashboard.putData("Swerve",
        builder -> {
            builder.addDoubleProperty(
                "Front Left Angle", () -> FL.getPosition().angle.getRadians(), null);
            builder.addDoubleProperty(
                "Front Left Velocity", () -> FL.getState().speedMetersPerSecond, null);
  
            builder.addDoubleProperty(
                "Front Right Angle", () -> FR.getPosition().angle.getRadians(), null);
            builder.addDoubleProperty(
                "Front Right Velocity", () ->FR.getState().speedMetersPerSecond, null);
  
            builder.addDoubleProperty(
                "Back Left Angle", () -> BL.getPosition().angle.getRadians(), null);
            builder.addDoubleProperty(
                "Back Left Velocity", () -> BL.getState().speedMetersPerSecond, null);
  
            builder.addDoubleProperty(
                "Back Right Angle", () -> BR.getPosition().angle.getRadians(), null);
            builder.addDoubleProperty(
                "Back Right Velocity", () -> BR.getState().speedMetersPerSecond, null);
  
            builder.addDoubleProperty(
                "Robot Angle", () -> getPose().getRotation().getRadians(), null);

            builder.setSmartDashboardType("SwerveDrive");
          });
    }


    /**
     * Drive the Swerve drivetrain based on target speeds in the X and Y directions,
     * and a target rotation speed.
     * 
     * @param translation The target translation speeds. X is forward/backward, Y is
     *  left/right. How this is treated is based on the fieldRelative parameter.
     * 
     * @param rotation The target rotation speed of the robot.
     * 
     * @param fieldRelative If true, treat the translation parameter as referenced to the 
     *  field coordinate system, where X is along the field's longer axis and Y along the shorter.
     *  If false, treat the translation parameter as referenced to the robot itself, 
     *  with X being forward/backward and Y being left/right.
     * 
     * @param isOpenLoop If true, the drive motors will operate in open loop mode, setting a raw
     *  percentage duty cycle. If false, the drive motors will run in closed loop mode, using their
     *  internal PID and feedforward controllers to hold a precise velocity. This should be true
     *  for manual driving to give drivers more responsive control, and false for autonomous driving
     *  to accurately match target speeds.
     * 
     * @param flipByAlliance If true, the translation input will be rotated based on which alliance
     *  the robot is on. Usually, this should be true for driver control (since the driver is
     *  standing at the opposite end of the field), and false for all automatic control. 
     *  No effect if fieldRelative is false.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, boolean flipByAlliance){    
        // Final target speeds for swerve drive
        ChassisSpeeds targetSpeeds;

        if(!fieldRelative)
        {
            // Not field oriented, just use raw values
            targetSpeeds = new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation);
        }
        else
        {
            // Field-oriented drive
            Optional<Alliance> a = DriverStation.getAlliance();
            Rotation2d yaw;

            /*
             * By default, we need to add 180 degrees to make the input translation
             * make sense in the standard coordinate system.
             * However, if we want to flip based on our alliance (for driver control),
             * then we don't add 180 when on blue alliance to achieve the flip.
             */
            if(flipByAlliance && a.isPresent() && a.get() == Alliance.Blue)
            {
                yaw = getOdometryYaw();
            }
            else
            {
                // Required when not flipping or on red
                yaw = getOdometryYaw().plus(Rotation2d.k180deg);
            }

            // Calculate chassis target speeds from field oriented speeds
            targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                yaw);
        }

        /*
         * Correct for skew while driving caused by modeling the system as continuous
         * when it is actually being calculated at discrete points in time
         * (in a really fast loop)
        */
        targetSpeeds = ChassisSpeeds.discretize(targetSpeeds, 0.02);

        // Log target speeds in a nice visualizable way
        chasTargetPublisher.set(targetSpeeds);

        // Calculate what the individual swerve modules should do to achieve this ChassisSpeeds
        SwerveModuleState[] swerveModuleStates = 
            Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(targetSpeeds);
        
        // Tell the swerve modules what to do
        setModuleStates(swerveModuleStates, isOpenLoop);
    }

    /**
     * Drive the Swerve drivetrain using a premade ChassisSpeeds object. This method overload
     * is intended for use with tools like PathPlanner. ChassisSpeeds is assumed to be 
     * robot-relative, and the swerve drive will run in closed loop mode.
     * @param robotRelativeSpeeds
     */
    public void drive(ChassisSpeeds robotRelativeSpeeds){
        Translation2d translation = new Translation2d(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond);
        double rotation = robotRelativeSpeeds.omegaRadiansPerSecond;
        drive(translation, rotation, false, false, false);
    }

    /* Used by PathPlanner AutoBuilder */
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.Swerve.SWERVE_KINEMATICS.toChassisSpeeds(
            mSwerveMods[0].getState(),
            mSwerveMods[1].getState(),
            mSwerveMods[2].getState(),
            mSwerveMods[3].getState()
        );
    }

    /**
     * Set the desired target states of the swerve modules.
     * @param desiredStates Desired swerve module states
     * @param openLoop Whether to run drive motors in open or closed loop mode
     */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean openLoop){

        // If any of the modules are being commanded to go over their max speed, scale all speeds down
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

        // Log target module statuses in a nice visualizable way
        modTargetPublisher.set(desiredStates);

        // Send data to each module
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], openLoop);
        }
    }

    public Command brake() {
        return new RunCommand(() -> {
            setModuleStates(new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45))
            }, true);
        },this);
    }

    /**
     * @return XY of robot on field
     */
    public Pose2d getPose(){
        return swerveOdometry.getEstimatedPosition();
    }

    /**
     * Use to reset odometry to a certain known pose or to zero
     * @param pose Desired new pose
     */
    public void resetOdometry(Pose2d pose){
        swerveOdometry.resetPosition(getGyroYaw(), getPositions(), pose);
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /**
     * @return Swerve Module positions
     */
    public SwerveModulePosition[] getPositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Use to reset angle to certain known angle or to zero
     * @param angle Desired new angle
     */
    public void zeroYaw(){
        //gyro.zeroYaw();
        Optional<Alliance> a = DriverStation.getAlliance();
        swerveOdometry.resetRotation(
            a.isPresent() && a.get() == Alliance.Red ?
            Rotation2d.k180deg
            :
            Rotation2d.kZero
        );
    }

    public Rotation2d getGyroYaw(){
        return (Constants.Swerve.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public Rotation2d getOdometryYaw() {
        return swerveOdometry.getEstimatedPosition().getRotation();
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getPositions());
        
        resetToEdge();

        SmartDashboard.putNumber("Gyro Angle", getGyroYaw().getDegrees());
        SmartDashboard.putNumber("Odo Angle", getOdometryYaw().getDegrees());

        SwerveModuleState[] currentStatus = new SwerveModuleState[4];
        
        for(SwerveModule mod : mSwerveMods){
            currentStatus[mod.moduleNumber] = mod.getState();
        }

        modStatusPublisher.set(currentStatus);
        chasStatusPublisher.set(getRobotRelativeSpeeds());

        field.setRobotPose(getPose());
    }


    private void setupSysIDTests() {

        SysIdRoutine routine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                    (v) -> {
                        for(int j = 0; j < mSwerveMods.length; j++)
                        {
                            mSwerveMods[j].setVoltage(v.in(edu.wpi.first.units.Units.Volts));
                        }
                    }, 
                    (log) ->
                    {
                        for(int j = 0; j < mSwerveMods.length; j++)
                        {
                            mSwerveMods[j].logSysIDData(log);
                        }
                    }, 
                    this)
            );

            SmartDashboard.putData("SysID/drive/dyn_f", routine.dynamic(Direction.kForward));
            SmartDashboard.putData("SysID/drive/dyn_r", routine.dynamic(Direction.kReverse));
            SmartDashboard.putData("SysID/drive/quas_f", routine.quasistatic(Direction.kForward));
            SmartDashboard.putData("SysID/drive/quas_r", routine.quasistatic(Direction.kReverse));

    }

    public void resetToEdge() {
        if (getPose().getY() > Constants.FIELD_WIDTH) {
            swerveOdometry.resetPosition(getGyroYaw(), getPositions(), new Pose2d(getPose().getX(), Constants.FIELD_WIDTH, getOdometryYaw()));        // Need to replace getPose(), getPost gets the current position, we need the desired position in the field
          } else if (getPose().getY() < 0.0) {
            swerveOdometry.resetPosition(getGyroYaw(), getPositions(), new Pose2d(getPose().getX(), 0.0, getOdometryYaw()));      
          } if (getPose().getX() > Constants.FIELD_LENGTH) {
            swerveOdometry.resetPosition(getGyroYaw(), getPositions(), new Pose2d(Constants.FIELD_LENGTH, getPose().getY(), getOdometryYaw())); 
          } else if (getPose().getX() < 0.0) {
            swerveOdometry.resetPosition(getGyroYaw(), getPositions(), new Pose2d(0.0, getPose().getY(), getOdometryYaw())); 
          }
    }


}
