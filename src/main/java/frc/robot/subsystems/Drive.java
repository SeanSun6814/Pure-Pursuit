package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import path.Odometer;

public class Drive extends Subsystem {
    TalonSRX leftMaster, rightMaster;
    VictorSPX leftSlave, rightSlave;

    Encoder leftEncoder, rightEncoder;
    AHRS gyro;
    double gyroZero = 0;

    private static Drive instance;

    public static Drive getInstance() {
        if (instance == null)
            instance = new Drive();
        return instance;
    }

    private Drive() {
        initGyro();
        initEncoders();
        initTalons();
    }

    @Override
    protected void initDefaultCommand() {

    }

    public void drive(double left, double right) {
        // ramp
        right = ramp(right, prevRightMotor);
        left = ramp(left, prevLeftMotor);
        prevRightMotor = right;
        prevLeftMotor = left;
        leftMaster.set(ControlMode.PercentOutput, left);
        rightMaster.set(ControlMode.PercentOutput, right);
    }

    public void driveJoystick(double power, double turn) {
        power = applyDeadband(power);

        turn = applyDeadband(turn);

        double right = power * 0.7 + turn * 0.8 * 0.6;
        double left = power * 0.7 - turn * 0.8 * 0.6;

        // ramp
        right = ramp(right, prevRightMotor);
        left = ramp(left, prevLeftMotor);
        prevRightMotor = right;
        prevLeftMotor = left;
        // ramp end

        rightMaster.set(ControlMode.PercentOutput, left);
        leftMaster.set(ControlMode.PercentOutput, right);
    }

    private void initTalons() {
        rightMaster = new TalonSRX(1);
        leftMaster = new TalonSRX(2);

        rightSlave = new VictorSPX(3);
        leftSlave = new VictorSPX(4);
        rightMaster.setInverted(true);
        rightSlave.setInverted(true);
        rightSlave.follow(rightMaster);
        leftSlave.follow(leftMaster);

        leftMaster.setNeutralMode(NeutralMode.Brake);
        rightMaster.setNeutralMode(NeutralMode.Brake);
        leftSlave.setNeutralMode(NeutralMode.Brake);
        rightSlave.setNeutralMode(NeutralMode.Brake);

        leftMaster.configNeutralDeadband(0.04);
        rightMaster.configNeutralDeadband(0.04);
        leftSlave.configNeutralDeadband(0.04);
        rightSlave.configNeutralDeadband(0.04);

        // rightSlave = TalonSRXFactory.createPermanentSlaveTalon(4, 3);
    }

    private void initGyro() {
        try {
            // gyro = new AHRS(SPI.Port.kMXP);
            gyro = new AHRS(SPI.Port.kMXP, (byte) 200);
            // gyro = new AHRS(SerialPort.Port.kUSB);
            zeroGyro();
        } catch (RuntimeException ex) {
            System.out.println("Error instantiating navX-MXP:  " + ex.getMessage());
        }
    }

    private void initEncoders() {
        final double kDriveWheelDiameter = 0.152; // meters
        final double kDriveGearboxRatio = 1 / 1.0;
        final double kDriveRotation2Distance = (kDriveGearboxRatio) * (Math.PI * kDriveWheelDiameter);
        // final double kDriveRotation2Distance = (kDriveGearboxRatio) * (0.4616);
        final double kDrivePulse2Distance = kDriveRotation2Distance / 128;

        leftEncoder = new Encoder(2, 3, false, EncodingType.k4X);
        leftEncoder.setMaxPeriod(1); // regard motor as stopped if no movement for 0.2 seconds
        leftEncoder.setMinRate(0); // regard motor as stopped if distance per second < 10
        // gearbox ratio 1:49; 0.5 inch changing diameter TODO;
        // final double PulseToDistanceConst = (1 / 49) * Math.PI * 0.02; // rotations
        // -> meters
        leftEncoder.setDistancePerPulse(kDrivePulse2Distance); // the scaling constant that converts pulses
        // into distance
        leftEncoder.setSamplesToAverage(5); // used to reduce noise in period
        leftEncoder.reset();

        // ---------------------------------------------------

        rightEncoder = new Encoder(1, 0, false, EncodingType.k4X);
        rightEncoder.setMaxPeriod(1); // regard motor as stopped if no movement for 0.2 seconds
        rightEncoder.setMinRate(0); // regard motor as stopped if distance per second < 10
        // gearbox ratio 1:49; 0.5 inch changing diameter TODO;
        // final double PulseToDistanceConst = (1 / 49) * Math.PI * 0.02; // rotations
        // -> meters
        rightEncoder.setDistancePerPulse(kDrivePulse2Distance); // the scaling constant that converts pulses
        // into distance
        rightEncoder.setSamplesToAverage(5); // used to reduce noise in period
        rightEncoder.reset();
    }

    public double getEncoders() {
        return (leftEncoder.get() + rightEncoder.get()) / 2;
    }

    public double getGyro() {
        return gyro.getAngle() - gyroZero;
    }

    public void zeroEncoder() {
        leftEncoder.reset();
        rightEncoder.reset();
        Odometer.getInstance().reset();
    }

    public void resetSensors() {
        zeroGyro();
        zeroEncoder();
    }

    public void zeroGyro() {
        gyroZero = gyro.getAngle();
    }

    public double getLeftEncoder() {
        return leftEncoder.getDistance();
    }

    public double getRightEncoder() {
        return rightEncoder.getDistance();
    }

    private double applyDeadband(double value) {
        if (value > 0.04) {
            return value;
        } else if (value < -0.04) {
            return value;
        }
        return 0;
    }

    private double prevLeftMotor = 0, prevRightMotor = 0;
    private double maxAcceleration = 0.025; // 12/0.05;

    private double ramp(double value, double prevValue) {
        if (value - prevValue > maxAcceleration) {
            value += maxAcceleration;
        } else if (value - prevValue < -maxAcceleration) {
            value -= maxAcceleration;
        }
        return value;
    }
}
