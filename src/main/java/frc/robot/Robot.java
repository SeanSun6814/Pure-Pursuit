/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.math.BigDecimal;
import java.math.RoundingMode;

import Obj.MessageLevel;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commandGroups.LeftPath;
import frc.robot.subsystems.Drive;
import path.Odometer;

// public class Robot extends IterativeRobot {
public class Robot extends TimedRobot {

    Joystick joystick1 = new Joystick(0);
    Joystick joystick2 = new Joystick(1);

    Drive drive = Drive.getInstance();
    Odometer odometer = Odometer.getInstance();
    Logger log = Logger.getInstance();

    Command command;

    public void robotInit() {
        drive.resetSensors();
        command = new LeftPath();
        log.log("Robot Inited", "Robot", MessageLevel.Info);
    }

    @Override
    public void autonomousInit() {
        if (joystick1.getRawButton(1)) {
        } else {
            drive.resetSensors();
            log.log("SENSORS RESET!!!");
        }
        command.start();
    }

    @Override
    public void autonomousPeriodic() {
        updateOdometer();
        Scheduler.getInstance().run();
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
        updateOdometer();
        double power = -joystick1.getRawAxis(1);
        // double turn = joystick2.getRawAxis(0);
        double turn = joystick1.getRawAxis(4);

        drive.driveJoystick(power, turn);

        updateOdometer();

        updateSmartDashboard();
    }

    @Override
    public void testPeriodic() {
        updateOdometer();
    }

    private void updateOdometer() {
        odometer.update(drive.getLeftEncoder(), drive.getRightEncoder(), drive.getGyro());
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
        ;
        SmartDashboard.putNumber("Y", round(odometer.getY()));

        SmartDashboard.putNumber("Gyro", round(drive.getGyro()));

        SmartDashboard.putNumber("Left Encoder", round(drive.getLeftEncoder()));
        SmartDashboard.putNumber("Right Encoder", round(drive.getRightEncoder()));

        // log.log("POS: (" + round(odometer.getX()) + "; " + round(odometer.getY()));
        log.log("| Encoders left right: " + round(drive.getLeftEncoder()) + "; " + round(drive.getRightEncoder()));
    }
}
