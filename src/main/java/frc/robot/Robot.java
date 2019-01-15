/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.Arrays;

import Obj.MessageLevel;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drive;
import path.DriveMotorState;
import path.Odometer;
import path.Path;
import path.PathFollower;
import path.PathGenerator;
import path.Point;
import path.Waypoint;

// public class Robot extends IterativeRobot {
public class Robot extends TimedRobot {

  Joystick joystick1 = new Joystick(0);
  Joystick joystick2 = new Joystick(1);

  Drive drive = Drive.getInstance();
  Odometer odometer = Odometer.getInstance();

  public Logger log = Logger.getInstance();

  public void robotInit() {
    drive.resetSensors();
    log.log("Robot Inited", "Robot", MessageLevel.Info);
  }

  @Override
  public void autonomousInit() {
    if (joystick1.getRawButton(1)) {
    } else {
      drive.resetSensors();
      log.log("SENSORS RESET!!!");
    }
    initPath();
  }

  PathFollower pathFollower;
  Path path = null;
  double dt = 0.05;

  private void initPath() {
    // double angle = 0;
    Point pos = new Point();
    double maxVel = 0.3; // 6 feet/s
    double maxAcc = 0.1; // m/sec every sec
    double spacing = 0.1, maxAngVel = 0.6; // 3 in simu
    double lookAheadDistance = 0.1524 * 3/* 6 inches */;
    double trackWidth = 0.8;// 0.5842; // 23 inches
    double targetTolerance = 0.1;// m
    // double maxVel = 250, maxAcc = 70, spacing = 6, maxAngVel = 6;
    // double lookAheadDistance = 12;

    // double kLeftV = 1, kLeftA = 0, kLeftP = 0, kRightV = 1, kRightA = 0, kRightP
    // = 0, kSmooth = 0.8, kTolerance = 0.001;

    // generate path
    PathGenerator pathGenerator = new PathGenerator();
    // path = pathGenerator
    // .calculate(new Path(spacing, maxVel, maxAcc, maxAngVel, Arrays.asList(new
    // Waypoint(0, 0), new Waypoint(0, 2))));
    // path = pathGenerator.calculate(new Path(spacing, maxVel, maxAcc, maxAngVel,
    // Arrays.asList(new Waypoint(0, 0), new Waypoint(0, 2), new Waypoint(-2, 2),
    // new Waypoint(-2, 6.5))));
    // path = pathGenerator.calculate(new Path(spacing, maxVel, maxAcc, maxAngVel,
    // Arrays.asList(new Waypoint(0, 0), new Waypoint(0, 4), new Waypoint(-2, 4),
    // new Waypoint(-2, 0))));
    path = pathGenerator.calculate(new Path(spacing, maxVel, maxAcc, maxAngVel,
        Arrays.asList(new Waypoint(0, 0), new Waypoint(0, -4), new Waypoint(-2, -4), new Waypoint(-2, 0))));

    log.log("Path: [" + path.waypoints.size() + "]");
    log.log(path);
    log.log("__________________________\n\n");
    // end generate path

    pathFollower = new PathFollower(path, lookAheadDistance, trackWidth, targetTolerance);
    pos = path.waypoints.get(0).p.copy();
    this.odometer.setX(pos.x);
    this.odometer.setY(pos.y);
  }

  @Override
  public void autonomousPeriodic() {
    updateOdometer();
    DriveMotorState driveMotorState = pathFollower.update(odometer.getPoint(), drive.getGyro(), dt);

    updateSmartDashboard();

    SmartDashboard.putNumber("Left Vel", round(driveMotorState.leftVel));
    SmartDashboard.putNumber("Right Vel", round(driveMotorState.rightVel));

    log.log("Left Vel" + round(driveMotorState.leftVel));
    log.log("Right Vel" + round(driveMotorState.rightVel));

    log.log("_____________________________________\n");
    drive.drive(driveMotorState.leftVel / 1.8288 * 2, driveMotorState.rightVel / 1.8288 * 2);
  }

  @Override
  public void teleopInit() {
    if (joystick1.getRawButton(1)) {
      drive.resetSensors();
      log.log("SENSORS RESET!!!");
    }
  }

  @Override
  public void teleopPeriodic() { // run every 50ms, 20Hz
    double power = -joystick1.getRawAxis(1);
    double turn = joystick2.getRawAxis(0);
    // double turn = joystick1.getRawAxis(0);

    drive.driveJoystick(power, turn);

    updateOdometer();

    updateSmartDashboard();
  }

  @Override
  public void testPeriodic() {

  }

  private void updateOdometer() {
    odometer.update(drive.getLeftEncoder(), drive.getRightEncoder(), drive.getGyro());

    // log.log("(" + odometer.getX() + ", " + odometer.getY() + ") / " +
    // getGyro() + " | "
    // + leftEncoder.getDistance() + ", " + rightEncoder.getDistance());
  }

  @Override
  public void disabledPeriodic() {
    if (joystick1.getRawButton(1)) {
      drive.resetSensors();
      log.log("SENSORS RESET!!!");
      log.log("DRIVE STATS:  GYRO: " + drive.getGyro() + "| Encoders left right: " + drive.getLeftEncoder() + "; "
          + drive.getRightEncoder());
    }
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

  private void updateSmartDashboard() {
    SmartDashboard.putNumber("X", round(odometer.getX()));
    SmartDashboard.putNumber("Y", round(odometer.getY()));

    SmartDashboard.putNumber("Gyro", round(drive.getGyro()));

    SmartDashboard.putNumber("Left Encoder", round(drive.getLeftEncoder()));
    SmartDashboard.putNumber("Right Encoder", round(drive.getRightEncoder()));

    log.log("POS: (" + round(odometer.getX()) + "; " + round(odometer.getY()));
    log.log("/ GYRO: " + round(drive.getGyro()));
    log.log("| Encoders left right: " + round(drive.getLeftEncoder()) + "; " + round(drive.getRightEncoder()));
  }
}
