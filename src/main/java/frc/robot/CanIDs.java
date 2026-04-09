// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

/** Constants file for CAN IDs */
public final class CanIDs {

    
    // Swerve CAN IDs
    public static final class SwerveModules {
        // FL
        public static final SwerveModuleCanIDs MOD0 = 
            new SwerveModuleCanIDs(10, 11);

        // FR
        public static final SwerveModuleCanIDs MOD1 = 
            new SwerveModuleCanIDs(20, 21);

        // BL
        public static final SwerveModuleCanIDs MOD2 = 
            new SwerveModuleCanIDs(40, 41);

        // BR
        public static final SwerveModuleCanIDs MOD3 = 
            new SwerveModuleCanIDs(30, 31);
    }

    public static final int SHOOT_MOTOR_1_ID = 57;
    public static final int SHOOT_MOTOR_2_ID = 58;
    public static final int ROTATE_MOTOR_ID = 55;
    public static final int HOOD_MOTOR_ID = 56;
    public static final int INTAKE_MOTION_MOTOR_ID = 50;
    public static final int INTAKE_PIVOT_MOTOR_ID = 51;
    public static final int INDEX_MOTOR_ID = 52;
    public static final int LATERAL_TURDEXER_MOTOR_ID = 53;
    public static final int MEDIAL_TURDEXER_MOTOR_ID = 54;


    // TODO Put constants for your other CAN devices here
    // public static final int EXAMPLE_MOTOR_ID = 45;



    /**
     * Utility class to hold all the CAN IDs for a swerve module.
     */
    public static final class SwerveModuleCanIDs
    {
        public final int driveMotorID;
        public final int angleMotorID;

        public SwerveModuleCanIDs(int driveMotorID, int angleMotorID)
        {
            this.angleMotorID = angleMotorID;
            this.driveMotorID = driveMotorID;
        }
    }
    

    // Map of CAN IDs for URCL
    // TODO Keep this up to date with the actual CAN IDs so logs make sense!
    public static final Map<Integer, String> URCL_IDS = Map.ofEntries(
          Map.entry(SwerveModules.MOD0.driveMotorID, "Swerve/FL/Drive"),
          Map.entry(SwerveModules.MOD0.angleMotorID, "Swerve/FL/Angle"),
          Map.entry(SwerveModules.MOD1.driveMotorID, "Swerve/FR/Drive"),
          Map.entry(SwerveModules.MOD1.angleMotorID, "Swerve/FR/Angle"),
          Map.entry(SwerveModules.MOD2.driveMotorID, "Swerve/BR/Drive"),
          Map.entry(SwerveModules.MOD2.angleMotorID, "Swerve/BR/Angle"),
          Map.entry(SwerveModules.MOD3.driveMotorID, "Swerve/BL/Drive"),
          Map.entry(SwerveModules.MOD3.angleMotorID, "Swerve/BL/Angle"),
          Map.entry(INTAKE_MOTION_MOTOR_ID, "Intake/Motion"),
          Map.entry(INDEX_MOTOR_ID, "Index/Index"),
          Map.entry(LATERAL_TURDEXER_MOTOR_ID, "Index/LateralTurdex"),
          Map.entry(MEDIAL_TURDEXER_MOTOR_ID, "Index/MedialTurdex"),
          Map.entry(ROTATE_MOTOR_ID, "Turret/Rotate"),
          Map.entry(HOOD_MOTOR_ID, "Turret/Hood"),
          Map.entry(SHOOT_MOTOR_1_ID, "Turret/Shoot1"),
          Map.entry(SHOOT_MOTOR_2_ID, "Turret/Shoot2"),
          Map.entry(INTAKE_PIVOT_MOTOR_ID, "Intake/Pivot")
      );

}
