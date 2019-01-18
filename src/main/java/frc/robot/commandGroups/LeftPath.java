/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandGroups;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.FollowPath;
import path.RobotPathConfig;
import path.Units;
import path.Waypoint;

public class LeftPath extends CommandGroup {

        public LeftPath() {
                boolean reset, reverse;
                List<Waypoint> waypoints;

                reset = true;
                reverse = false;

                waypoints = Arrays.asList(new Waypoint(Units.in2m(116.75), Units.in2m(66)),
                                new Waypoint(Units.in2m(116.75), Units.in2m(66) + 0.5),
                                new Waypoint(Units.in2m(150.12), Units.in2m(227.75) - 2),
                                new Waypoint(Units.in2m(150.12), Units.in2m(227.75)));

                addSequential(new FollowPath(reverse, reset, RobotPathConfig.getPracticeRobotConfig(), waypoints));

                reset = false;
                reverse = true;

                waypoints = Arrays.asList(new Waypoint(Units.in2m(150.12), Units.in2m(227.75)),
                                new Waypoint(Units.in2m(150.12), Units.in2m(227.75) - 1),
                                new Waypoint(Units.in2m(25.94), Units.in2m(0) + 2),
                                new Waypoint(Units.in2m(25.94), Units.in2m(0)));

                addSequential(new FollowPath(reverse, reset, RobotPathConfig.getPracticeRobotConfig(), waypoints));

                reset = false;
                reverse = false;

                waypoints = Arrays.asList(new Waypoint(Units.in2m(25.94), Units.in2m(0)),
                                new Waypoint(Units.in2m(25.94), Units.in2m(0) + 1),
                                // new Waypoint(Units.in2m(74), Units.in2m(250)),
                                new Waypoint(Units.in2m(138) - 1.7, Units.in2m(260.75)),
                                new Waypoint(Units.in2m(138), Units.in2m(260.75)));

                addSequential(new FollowPath(reverse, reset, RobotPathConfig.getPracticeRobotConfig(), waypoints));

        }
}

/**
 * public OnePath() { // boolean reset = true, reverse = false;
 * 
 * // addSequential(new FollowPath(reverse, reset, //
 * RobotPathConfig.getPracticeRobotConfig(), // Arrays.asList(new Waypoint(0,
 * 0), new Waypoint(0, 2), new Waypoint(-2, 2), // new Waypoint(-2, 4))));
 * 
 * // reset = false; // reverse = true;
 * 
 * // addSequential(new FollowPath(reverse, reset, //
 * RobotPathConfig.getPracticeRobotConfig(), // Arrays.asList(new Waypoint(-2,
 * 4), new Waypoint(-2, 2), new Waypoint(0, 2), // new Waypoint(0, 0))));
 * 
 * boolean reset = true, reverse = false;
 * 
 * addSequential(new FollowPath(reverse, reset,
 * RobotPathConfig.getPracticeRobotConfig(), Arrays.asList(new Waypoint(0, 0),
 * new Waypoint(0, 4))));
 * 
 * reset = false; reverse = true;
 * 
 * addSequential(new FollowPath(reverse, reset,
 * RobotPathConfig.getPracticeRobotConfig(), Arrays.asList(new Waypoint(0, 4),
 * new Waypoint(0, 2), new Waypoint(-2, 2))));
 * 
 * reset = false; reverse = false;
 * 
 * addSequential(new FollowPath(reverse, reset,
 * RobotPathConfig.getPracticeRobotConfig(), Arrays.asList(new Waypoint(-2, 2),
 * new Waypoint(0, 2), new Waypoint(0, 0))));
 * 
 * }
 */