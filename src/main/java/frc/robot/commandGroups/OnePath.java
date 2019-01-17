/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandGroups;

import java.util.Arrays;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.FollowPath;
import path.RobotPathConfig;
import path.Waypoint;

public class OnePath extends CommandGroup {

  public OnePath() {
    boolean reset = true, reverse = false;

    addSequential(new FollowPath(reverse, reset, RobotPathConfig.getPracticeRobotConfig(),
        Arrays.asList(new Waypoint(0, 0), new Waypoint(0, 2), new Waypoint(2, 2))));
    // addSequential(new FollowPath(reverse, reset,
    // Arrays.asList(new Waypoint(0, 0), new Waypoint(0, 4), new Waypoint(-2, 4),
    // new Waypoint(-2, 2),
    // new Waypoint(0, 2), new Waypoint(0, 4), new Waypoint(-2, 4), new Waypoint(-2,
    // 2), new Waypoint(2, 2))));

    reset = false;
    reverse = true;

    addSequential(new FollowPath(reverse, reset, RobotPathConfig.getPracticeRobotConfig(),
        Arrays.asList(new Waypoint(2, 2), new Waypoint(0, 2), new Waypoint(0, 4))));
  }
}
