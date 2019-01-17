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

    private static RobotPathConfig practiceRobotConfig;

    public static RobotPathConfig getPracticeRobotConfig() {
        if (practiceRobotConfig == null) {
            practiceRobotConfig = new RobotPathConfig();
            practiceRobotConfig.maxVel = Units.ft2m(1.4);// is this really that slow? TODO:
            practiceRobotConfig.maxAcc = Units.ft2m(0.3); // m/sec every sec
            practiceRobotConfig.spacing = Units.ft2m(1);
            practiceRobotConfig.maxAngVel = 1.5; // radians per second
            practiceRobotConfig.lookAheadDistance = Units.ft2m(2);
            practiceRobotConfig.trackWidth = Units.in2m(31.5);// 23 inches
            practiceRobotConfig.targetTolerance = Units.in2m(4);// m
            practiceRobotConfig.physicalMaxVel = Units.ft2m(12);
        }
        return practiceRobotConfig;
    }

}