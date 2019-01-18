package path;

public class RobotPathConfig {

    public double maxVel;// max vel the robot will follow at
    public double physicalMaxVel; // max vel hardware can support
    public double maxAcc;// max acceleration wanted to follow at
    public double maxAngVel;// max angular accel wanted to follow at
    public double spacing; // path waypoint spacing
    public double lookAheadDistance;// distance to find lookahead point
    public double trackWidth;// track width between left, right wheels (make 1-3 inches bigger)
    public double targetTolerance; // how close to the target is considered finished
    public double kP; // porportional constant for PIDVA motor output
    public double kV; // velocity constant for PIDVA motor output
    public double kA; // acceleration constant for PIDVA motor output

    private static RobotPathConfig practiceRobotConfig;

    public static RobotPathConfig getPracticeRobotConfig() {
        if (practiceRobotConfig == null) {
            practiceRobotConfig = new RobotPathConfig();
            practiceRobotConfig.maxVel = Units.ft2m(9); // 7 before
            practiceRobotConfig.maxAcc = Units.ft2m(3); // m/sec every sec
            practiceRobotConfig.spacing = Units.ft2m(1);
            practiceRobotConfig.maxAngVel = 2.5; // radians per second
            practiceRobotConfig.lookAheadDistance = Units.ft2m(1);
            practiceRobotConfig.trackWidth = Units.in2m(25);// 23 inches
            practiceRobotConfig.targetTolerance = Units.in2m(4);// m
            practiceRobotConfig.physicalMaxVel = Units.ft2m(12);
            practiceRobotConfig.kV = 1 / practiceRobotConfig.physicalMaxVel;
            practiceRobotConfig.kA = 0;
            practiceRobotConfig.kP = 0;
        }
        return practiceRobotConfig;
    }

}