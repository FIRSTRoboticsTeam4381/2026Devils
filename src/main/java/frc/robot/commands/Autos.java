// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.fasterxml.jackson.databind.deser.impl.PropertyValue;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public final class Autos {;
    // TODO register commands in subsystem constructors using NamedCommands.registerCommand()

    // Test autonomous mode
    public static PreviewAuto testAuto(){
        return new PreviewAuto("Test");
    }

    /**
     * Blank Autonomous to be used as default dashboard option
     * @return Autonomous command
     */
    public static PreviewAuto none(){
        return new PreviewAuto(Commands.none());
    }

    public static PreviewAuto LeftAuto(){
        return new PreviewAuto("LeftAuto");
    }
    public static PreviewAuto SimpleLeftAuto(){
        return new PreviewAuto("SimpleLeftAuto");
    }
    public static PreviewAuto SimpleRightAuto(){
        return new PreviewAuto("SimpleRightAuto");
    }
    public static PreviewAuto MiddleAuto(){
        return new PreviewAuto("MiddleAuto");
    }

    public static PreviewAuto MoreMiddleAuto(){
        return new PreviewAuto("MoreMiddleAuto");
    }
    public static PreviewAuto RightOPAuto(){
        return new PreviewAuto("RightOP");
    }
    public static PreviewAuto LeftOSOSAuto(){
        return new PreviewAuto("LeftOSOS");
    }

    public static PreviewAuto RightOSOSAuto(){
        return new PreviewAuto("RightOSOS");
    }

    public static PreviewAuto BallScatter() {
        return new PreviewAuto("BallScatter");
    }

    public static PreviewAuto EvilBallScatter() {
        return new PreviewAuto("EvilBallScatter");
    }

    public static PreviewAuto LeftDepotAuto() {
        return new PreviewAuto("Left&DepotAuto");
    }

    // TODO add pathplanner autos here. Example:
    //public static PreviewAuto Front3Note(){
    //    return new PreviewAuto("Front3NoteAuto");
    //}

    /* If you want to make a more complex auto using commands,
    *  PreviewAuto can also accept (Command, String), which will
    *  run Command while still showing a path preview for the path
    *  with filename String.
    */ 

    public static class PreviewAuto {
        public Command auto;
        private List<PathPlannerPath> paths;

        public void showPreview() {
            if(paths != null)
            {
                ArrayList <Pose2d> preview = new ArrayList<>();
                Optional<Alliance> alliance = DriverStation.getAlliance();
                                    

                for(PathPlannerPath p : paths)
                {
                    if(alliance.isPresent() && alliance.get() == Alliance.Red)
                        preview.addAll(p.flipPath().getPathPoses());
                    else
                        preview.addAll(p.getPathPoses());
                }
            
                
                RobotContainer.getRobot().swerve.field.getObject("path").setPoses(preview);
            }
        }

        public PreviewAuto(Command a) {
            auto = a;
        }

        public PreviewAuto(String s) {
            auto = new PathPlannerAuto(s);

            try
            {
                paths = PathPlannerAuto.getPathGroupFromAutoFile(s);
            }
            catch(Exception e)
            {
                e.printStackTrace();
                DriverStation.reportError("Failed to load autonomous from PathPlanner file!", false);
            }
        }

        public PreviewAuto(Command c, String s) {
            auto = c;

            try
            {
                paths = PathPlannerAuto.getPathGroupFromAutoFile(s);
            }
            catch(Exception e)
            {
                e.printStackTrace();
                DriverStation.reportError("Failed to load autonomous from PathPlanner file!", false);
            }
        }

    }

}
