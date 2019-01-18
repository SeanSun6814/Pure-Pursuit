/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Drive;
import path.DriveMotorState;
import path.Odometer;
import path.Path;
import path.PathFollower;
import path.PathGenerator;
import path.Point;
import path.RobotPathConfig;
import path.Waypoint;

public class FollowPath extends CommandBase {

    PathFollower pathFollower;
    Odometer odometer = Odometer.getInstance(); // only decleared to OBTAIN coord
    Drive drive = Drive.getInstance();

    Path path = null;
    RobotPathConfig config;
    double prevTimestamp = 0;

    boolean reversed;
    boolean requestReset;

    public FollowPath(boolean reversed, boolean reset, RobotPathConfig robotConfig, List<Waypoint> waypoints) {
        requires(Drive.getInstance());
        requestReset = reset;
        this.reversed = reversed;
        this.config = robotConfig;
        initPath(waypoints);
    }

    protected void initialize() {
        log("Starting path");
        if (requestReset) {
            log("requested reset, requested");
            drive.resetSensors();
            Point pos = path.waypoints.get(0).p;
            odometer.setX(pos.x);
            odometer.setY(pos.y);
        }
        prevTimestamp = Timer.getFPGATimestamp();
    }

    private void initPath(List<Waypoint> waypoints) {

        // generate path
        PathGenerator pathGenerator = new PathGenerator();

        path = pathGenerator
                .calculate(new Path(config.spacing, config.maxVel, config.maxAcc, config.maxAngVel, waypoints));

        log("Path: [" + path.waypoints.size() + "]");
        for (int i = 0; i < path.waypoints.size(); i++) {
            log("path", path.waypoints.get(i));
        }
        log("__________________________\n\n");
        // end generate path
        pathFollower = new PathFollower(path, config.lookAheadDistance, config.trackWidth, config.targetTolerance,
                reversed);

    }

    @Override
    protected void execute() {
        double dt = Timer.getFPGATimestamp() - prevTimestamp; // seconds
        prevTimestamp = Timer.getFPGATimestamp();

        log("dt", dt);

        DriveMotorState driveMotorState = pathFollower.update(odometer.getPoint(), drive.getGyro(), dt);

        log("gyro", round(drive.getGyro()));
        log("_____________________________________\n");

        drive.drive(driveMotorState.leftVel / config.physicalMaxVel, driveMotorState.rightVel / config.physicalMaxVel);
        // divide because the max/min motor output is -1 to 1
    }

    public static double round(double value) {
        int places = 2;
        // return value;
        if (places < 0)
            throw new IllegalArgumentException();

        BigDecimal bd = new BigDecimal(value);
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return pathFollower.done;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        log("Path Follow Command done.");
        drive.drive(0, 0);
    }

    @Override
    protected void interrupted() {
        end();
    }
}
