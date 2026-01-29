// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Default deadband for joystics
    public static final double STICK_DEADBAND = 0.15;

    // Max field dimensions, used as a sanity check by odometry and vision
    public static final double FIELD_WIDTH = 8.07;
    public static final double FIELD_LENGTH = 16.54;

    public static final double MIN_VISION_ESTIMATE_Z = -0.5;
    public static final double MAX_VISION_ESTIMATE_Z = 0.5;

    

    public static final class Swerve{
        public static final boolean INVERT_GYRO = true; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(24.75); 
        public static final double WHEEL_BASE = Units.inchesToMeters(18.75); 
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0); // This may reduce as wheel tread wears down!
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        public static final double driveGearRatio = (425.0 / 72.0);  //Mk4i L2+

        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));
        
        // PathPlanner PID params
        public static final PPHolonomicDriveController HOLOMONIC_CONFIG = new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        );

        /* Swerve Profiling Values */
        public static final double MAX_SPEED = 6.126; //meters per second
        public static final double MAX_ANGULAR_VELOCITY = 11.5; //TODO this can probably be increased

        /* Motor Inverts */
        public static final boolean driveMotorInvert = true;
        public static final boolean angleMotorInvert = true;


        // PID constants for on-RIO PID control of the robot's position
        // Currently used by AutoCorrect and SnapToPosition
        // These will be different from the values used by PathPlanner!
        public static final PIDConstants TRANSLATION_PID = 
            new PIDConstants(4.5, 0, 0);

        public static final PIDConstants ROTATION_PID = 
            new PIDConstants(0.04, 0, 0);
    }
}
