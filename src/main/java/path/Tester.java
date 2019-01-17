package path;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.HeadlessException;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.Arrays;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.JFrame;
import javax.swing.JPanel;

/**
 * This is a simlator made for the pure pursuit algorithm. It has a robot object
 * that stores all the robot properties, and a JPanel class that updates those
 * properties onto the screen.
 * 
 * HOWEVER, to use, first comment out (or replace with sysout) all the log()
 * commands in path generator and path follower. This is because
 */

public class Tester extends JFrame {

	private static final double robotDt = 50; // each update is how many ms for the robot, make this bigger to make the
												// robot go slower.
	private static final long dt = 50; // interval of this program actually updating
	Robot robot = new Robot(robotDt);

	public static void main(String[] args) throws HeadlessException {
		new Tester();
	}

	public Tester() {
		Timer timer = new Timer();
		robot.update();
		timer.schedule(new Helper(robot), 3000, dt); // delay 3s at the beginning
		setSize(1000, 800);
		setLocationRelativeTo(null);
		setDefaultCloseOperation(EXIT_ON_CLOSE);
		setContentPane(new DrawArea(robot));
		setVisible(true);
	}

}

class DrawArea extends JPanel {
	// java.awt.Point p = new java.awt.Point(300, 200);
	private Robot r;

	public DrawArea(Robot robot) {
		this.r = robot;
	}

	@Override
	protected void paintComponent(Graphics g) {

		// set background
		super.paintComponent(g);

		// make stroke thicker
		((Graphics2D) g).setStroke(new BasicStroke(7));

		drawPath(g);

		// robot
		g.setColor(Color.blue);
		g.drawArc(scale(r.pos.x) - 25, scale(r.pos.y) - 25, 50, 50, 0, 360);

		// direction
		g.drawLine(scale(r.pos.x), scale(r.pos.y), scale(r.pos.x + 10 * Math.cos(Math.toRadians(r.angle))),
				scale(r.pos.y + 10 * Math.sin(Math.toRadians(r.angle))));

		// lookahead circle
		g.setColor(Color.orange);
		if (r.pathFollower.onPath)
			g.drawArc(scale(r.pos.x - r.lookAheadDistance), scale(r.pos.y - r.lookAheadDistance),
					scale(r.lookAheadDistance * 2), scale(r.lookAheadDistance * 2), 0, 360);
		else
			g.drawArc(scale(r.pos.x - r.lookAheadDistance * 2), scale(r.pos.y - r.lookAheadDistance * 2),
					scale(r.lookAheadDistance * 2 * 2), scale(r.lookAheadDistance * 2 * 2), 0, 360);

		// lookahead point
		g.setColor(Color.green);
		if (r.pathFollower.prevLookAheadPoint != null)
			g.drawLine(scale(r.pos.x), scale(r.pos.y), scale(r.pathFollower.prevLookAheadPoint.x),
					scale(r.pathFollower.prevLookAheadPoint.y));

		// closest point
		g.setColor(Color.red);
		g.drawLine(scale(r.pos.x), scale(r.pos.y), scale(r.pathFollower.closestpoint.p.x),
				scale(r.pathFollower.closestpoint.p.y));

		// draw velocity
		g.setColor(Color.black);
		g.setFont(new Font("Calibri", Font.PLAIN, 40));
		g.drawString("Speed = " + String.valueOf(round((r.leftVel + r.rightVel) / 2, 2)),
				scale(r.pathFollower.closestpoint.p.x) + 70, scale(r.pathFollower.closestpoint.p.y) + 50);

		// texts
		g.setFont(new Font("Calibri", Font.PLAIN, 40));
		g.setColor(Color.black);
		g.drawString("Position: (" + r.pos.toString() + ")", 50, 50);
		g.drawString("Direction: " + String.valueOf(round(r.angle + 90, 2)), 50, 100);
		// System.out.println(scale(r.pos.y) + scale(r.pos.x));
		this.repaint();
	}

	protected void drawPath(Graphics g) {
		List<Waypoint> waypoints = r.path.waypoints;
		g.setColor(Color.red);

		// connects the path in red
		for (int i = 0; i < waypoints.size() - 1; i++) {
			g.drawLine(scale(waypoints.get(i).p.x), scale(waypoints.get(i).p.y), scale(waypoints.get(i + 1).p.x),
					scale(waypoints.get(i + 1).p.y));
		}

		// draw the single waypoints in black to debug path generation.
		((Graphics2D) g).setStroke(new BasicStroke(4));
		g.setColor(Color.black);
		for (int i = 0; i < waypoints.size() - 1; i++) {
			g.drawArc(scale(waypoints.get(i + 1).p.x), scale(waypoints.get(i + 1).p.y), 5, 5, 0, 360);
		}
		((Graphics2D) g).setStroke(new BasicStroke(7));

	}

	/**
	 * scale the units of pure pursuit up to more pixels, so it's bigger and easier
	 * to see
	 */
	public int scale(double x) {
		return (int) x * 100 + 300;
	}

	// rounding!! makes 16 decimal place numbers readable
	public static double round(double value, int places) {
		// return value;
		if (places < 0)
			throw new IllegalArgumentException();

		BigDecimal bd = new BigDecimal(value);
		bd = bd.setScale(places, RoundingMode.HALF_UP);
		return bd.doubleValue();
	}

}

class Robot {
	double dt; // stores what the delta time of each update is (this is set by user)
	public double angle = 0 + 90; // stores what angle the robot is at (deg)
	public Point pos = new Point();// stores the coordinate of the robot position (init at (0,0))
	public double leftVel = 0, rightVel = 0; // stores the left and right velocities of the drivetrain motors
	public double leftEncoder = 0, rightEncoder = 0, prevLeftEncoder = 0, prevRightEncoder = 0; // store encoder values
																								// and to calculate
																								// delta encoder values
	public double maxVel = 1.8288, maxAcc = 1, spacing = 10, maxAngVel = 15; // define maxes, spacing is used for path
																				// generation
	public double lookAheadDistance = 0.1524, trackWidth = 0.5842; // used for path following
	public double targetTolerance = lookAheadDistance;// used to determine when to stop

	// double maxVel = 250, maxAcc = 70, spacing = 6, maxAngVel = 6;
	// public double lookAheadDistance = 12;

	public Odometer odometer = Odometer.getInstance();
	public Path path = null;
	public double kLeftV = 1, kLeftA = 0, kLeftP = 0, kRightV = 1, kRightA = 0, kRightP = 0;
	public PathFollower pathFollower;
	public boolean reversed = false;

	public Robot(double dt) {
		this.dt = dt / 1000;
		generatePath();
		// path =new Path(spacing, maxVel, maxAcc, maxAngVel,
		// Arrays.asList(new Waypoint(0, 350,50), new Waypoint(100, 350,50), new
		// Waypoint(150, 300,50),
		// new Waypoint(150, 200,50), new Waypoint(200, 150,50), new Waypoint(300,
		// 150,50)));
		pathFollower = new PathFollower(path, lookAheadDistance, trackWidth, targetTolerance, reversed);
		pos = path.waypoints.get(0).p.copy();
		// pos = new Point(700, 100);
		// pos = new Point(80, 100);
		this.odometer.setX(pos.x);
		this.odometer.setY(pos.y);
	}

	public void generatePath() {
		PathGenerator p = new PathGenerator();

		// path = p.generate(kSmooth, kTolerance,
		// new Path(spacing, maxVel, maxAcc, maxAngVel, Arrays.asList(new Waypoint(1,
		// 1),
		// new Waypoint(5, 1),
		// new Waypoint(9, 12), new Waypoint(12, 9), new Waypoint(15, 6), new
		// Waypoint(19, 1))));

		// path = p.calculate(
		// new Path(spacing, maxVel, maxAcc, maxAngVel, Arrays.asList(new Waypoint(10,
		// 10),
		// new Waypoint(50, 10),
		// new Waypoint(90, 120), new Waypoint(120, 90), new Waypoint(150, 60), new
		// Waypoint(190, 10))));
		// path = p.calculate(new Path(spacing, maxVel, maxAcc, maxAngVel,
		// Arrays.asList(new Waypoint(50, 50), new Waypoint(100, 70), new Waypoint(300,
		// 130),
		// new Waypoint(150, 200), new Waypoint(50, 270), new Waypoint(400, 400))));
		// path = p.calculate(new Path(spacing, maxVel, maxAcc, maxAngVel,
		// Arrays.asList(new Waypoint(0, 50),
		// new Waypoint(100, 50), new Waypoint(160, 90), new Waypoint(160, 110))));
		path = p.calculate(new Path(spacing, maxVel, maxAcc, maxAngVel,
				Arrays.asList(new Waypoint(0, 0), new Waypoint(0, 2), new Waypoint(-2, 2))));
		// path = p.calculate(new Path(spacing, maxVel, maxAcc, maxAngVel,

		// Arrays.asList(new Waypoint(10, 10), new Waypoint(100, 70), new Waypoint(300,
		// 130),
		// new Waypoint(150, 200), new Waypoint(50, 270), new Waypoint(400, 400))));
		// path = p.calculate(new Path(spacing, maxVel, maxAcc, maxAngVel,
		// Arrays.asList(new Waypoint(50, 250), new Waypoint(200, 100), new
		// Waypoint(350, 250),
		// new Waypoint(350, 650), new Waypoint(200, 800), new Waypoint(50, 650), new
		// Waypoint(50, 300))));

		// path = p.generate(kSmooth, kTolerance,
		// new Path(spacing, maxVel, maxAcc, maxAngVel,
		// Arrays.asList(new Waypoint(0, 350), new Waypoint(100, 350), new Waypoint(150,
		// 300),
		// new Waypoint(150, 200), new Waypoint(200, 150), new Waypoint(300, 150))));
		System.out.println("Path: [" + path.waypoints.size() + "]");
		System.out.println(path);
		System.out.println("__________________________\n\n");
	}

	public void updateMotor(DriveMotorState state, double dt) {
		double left = state.leftAcc * kLeftA + state.leftVel * kLeftV;
		double right = state.rightAcc * kRightA + state.rightVel * kRightV;
		double leftAcc = (left - leftVel) / dt, rightAcc = (right - rightVel) / dt;

		if (leftAcc > maxAcc * 1.3)
			leftAcc = maxAcc * 1.3;
		else if (leftAcc < -maxAcc * 1.3)
			leftAcc = -maxAcc * 1.3;

		if (rightAcc > maxAcc * 1.3)
			rightAcc = maxAcc * 1.3;
		else if (rightAcc < -maxAcc * 1.3)
			rightAcc = -maxAcc * 1.3;

		// leftVel = maxVel * (left - (Math.random() * 0.01) - leftAcc * 0.05);
		// rightVel = maxVel * (right - (Math.random() * 0.01) - rightAcc * 0.05);
		leftVel += leftAcc * dt;
		rightVel += rightAcc * dt;
	}

	public void updateEncoder(double dt) {
		prevLeftEncoder = leftEncoder;
		prevRightEncoder = rightEncoder;

		leftEncoder += dt * leftVel;
		rightEncoder += dt * rightVel;
	}

	// requires update encoder && update gyro
	public void updateOdometer(double dt) {
		odometer.update(leftEncoder, rightEncoder, angle);
		pos.x = odometer.getX();
		pos.y = odometer.getY();
	}

	public void updateGyro() {
		double deltaLeftEncoder = leftEncoder - prevLeftEncoder;
		double deltaRightEncoder = rightEncoder - prevRightEncoder;
		double deltaDistance = deltaRightEncoder - deltaLeftEncoder;

		angle += Math.toDegrees(deltaDistance / trackWidth);
	}

	public void update() {
		print();
		DriveMotorState driveMotorState = pathFollower.update(pos, angle, dt);// = new DriveMotorState(1, 0, 0.8, 0);
		updateMotor(driveMotorState, dt);
		updateEncoder(dt);
		updateGyro();
		updateOdometer(dt);
	}

	public void print() {
		System.out.println("________________________________________________");
		System.out.println("Robot Position: (" + pos.x + "," + pos.y + ")");
		System.out.println("Robot Heading: (" + angle + ")");
		System.out.println("Robot Encoders: (" + leftEncoder + "," + rightEncoder + ")");
	}
}

class Helper extends TimerTask {
	Robot robot;

	public Helper(Robot robot) {
		this.robot = robot;
	}

	public void run() {
		robot.update();
	}
}
