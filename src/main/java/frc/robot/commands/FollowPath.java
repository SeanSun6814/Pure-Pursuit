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
    double prevLeftVel = 0, prevRightVel = 0;
    double prevLeftEncoder = 0, prevRightEncoder = 0;

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

        // get acceleration
        double leftAcc = (driveMotorState.leftVel - prevLeftVel) / dt;
        double rightAcc = (driveMotorState.rightVel - prevRightVel) / dt;

        prevLeftVel = driveMotorState.leftVel;
        prevRightVel = driveMotorState.rightVel;

        // get actual velocity from encoders
        double actualLeftVel = (drive.getLeftEncoder() - prevLeftEncoder) / dt;
        double actualRightVel = (drive.getRightEncoder() - prevRightEncoder) / dt;

        prevLeftEncoder = drive.getLeftEncoder();
        prevRightEncoder = drive.getRightEncoder();

        double left = calculatePIDVAL(driveMotorState.leftVel, leftAcc, actualLeftVel, config.kP, config.kV, config.kA);
        double right = calculatePIDVAR(driveMotorState.rightVel, rightAcc, actualRightVel, config.kP, config.kV,
                config.kA);

        log("actualmotoroutput", left + "; " + right);
        log("endloop", "_____________________________________\n");
        drive.drive(left, right);
    }

    private double calculatePIDVAL(double velocity, double acceleration, double actualVelocity, double kP, double kV,
            double kA) {
        log("lpidp", kP * (velocity - actualVelocity));
        log("lpida", kA * acceleration);
        log("lpiderror", (velocity - actualVelocity));

        return kV * velocity + kA * acceleration + kP * (velocity - actualVelocity);
    }

    private double calculatePIDVAR(double velocity, double acceleration, double actualVelocity, double kP, double kV,
            double kA) {
        log("rpidp", kP * (velocity - actualVelocity));
        log("rpida", kA * acceleration);
        log("rpiderror", (velocity - actualVelocity));

        return kV * velocity + kA * acceleration + kP * (velocity - actualVelocity);
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
