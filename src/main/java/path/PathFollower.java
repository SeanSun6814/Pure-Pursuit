package path;

import java.math.BigDecimal;
import java.math.RoundingMode;

import Obj.ExecTimer;
import Obj.LogBase;
import Obj.MessageLevel;

public class PathFollower extends LogBase {
	private Path path;
	private double lookAheadDistance;// TODO: lookaheaddistance should vary based on robot velocity to make sharp
										// turns
	private final double trackWidth; // make this a tiny bit larger due to scrubing

	private double targetTolerance;

	int lastLookAheadPointPathIndex = 0;
	int lastClosestPointPathIndex = 0;
	Point prevLookAheadPoint = null;
	public boolean done;
	public int searchLimit;
	public boolean onPath = true;
	double prevDist2Target = Double.MAX_VALUE;

	double prevGeneralVel = 0;
	boolean reversed;

	/// test vars

	public Waypoint closestpoint = new Waypoint(0, 0);
	public double debugcurvature = 0.0001;

	public static void main(String[] args) {

	}

	public PathFollower(Path path, double lookAheadDistance, double trackWidth, double targetTolerance,
			boolean reversed) {
		setLogSenderName("PathFollower");
		this.lookAheadDistance = lookAheadDistance;
		this.trackWidth = trackWidth;
		this.path = path;
		this.targetTolerance = targetTolerance;
		this.done = false;
		this.reversed = reversed;
		searchLimit = (int) (lookAheadDistance / path.spacing * 2) + 1; // leave 2x margin, and ceil it
		log("searchLimit", searchLimit);
	}

	/**
	 * This is the main function of pure pursuit.
	 * 
	 * Basically: I already know the path, so if you can tell me the robot pose
	 * (coordinate + heading) & time interval, I can calculate how fast the motors
	 * should spin at. Use PID later on to make sure it happens.
	 * 
	 * @param robotPos A point that holds the robot current position
	 * @param gyro     A angle in degrees (double) front is 0 deg, clockwise is
	 *                 positive
	 * @param dt       The time interval between now and the last update (in
	 *                 seconds)
	 * @return left & right motor speed (wrapped in a class called
	 *         {@link DriveMotorState}
	 */
	public DriveMotorState update(Point robotPos, double gyro, double dt) {
		ExecTimer execTimer = new ExecTimer();

		if (reversed) {
			gyro += 180; // make it look like the robot is headed forwards, so the curvature calculation
							// can go correctly
		}

		gyro = Math.toRadians(gyro + 90);

		Waypoint waypoint = getInterpolatedWaypoint(robotPos);
		log("closestwaypoint", waypoint);
		double curvature = getCurvature(robotPos, gyro);

		// double angularVel = waypoint.v * curvature; // w=v/r

		// angularVel = clamp(angularVel, -path.maxAngleVel, path.maxAngleVel);
		// curvature = angularVel / waypoint.v;

		// curvature = clamp(curvature, -maxAngularVel, maxAngularVel);

		// if we are on the path, then the target velocity is accurate, however, when we
		// are slightly off the path or getting back onto the path, we shouldn't use the
		// path's velocity. In other words, when we are not exactly on the path, we
		// could have a different curvature than the pre-generated curvature. If our
		// runtime curvature is big, but our path here happens to be straight, we may be
		// going through a small turn on great speeds.
		// The solution to this is to use the runtime curvature (just found) to
		// calculate the max velocity for desired AngAccel. But we cannot ignore the
		// pre-slow-down that our pre-generated velocities give us. To satisfy both, we
		// take the minimum of the two.

		// If we are not on the path, however, we don't need to think about the path's
		// velocity at all since the small turns don't apply, we can just simply go as
		// fast as our runtime turning allows.

		// current max velocity could be calculated:
		// angVel = linearVel / radius, curvature = 1 / radius
		// linearVel = angVel / curvature
		double targetVelocity;
		if (onPath) {
			targetVelocity = Math.min(waypoint.v, path.maxAngleVel / Math.abs(curvature));
		} else {
			targetVelocity = path.maxAngleVel / Math.abs(curvature);
		}

		// The path generation solves the slow down of the robot, but the speed up ramp
		// is not done (this is to avoid the initial velocity being 0, because then the
		// robot would never start moving). Moreover, it cannot be inside the drive
		// class as a simple ramp because that output ramp dampens two things: (turning
		// acceleration, and general acceleration). True, it would make the speed up
		// process slower, but it would also make the TURNS more delayed and
		// uncontrollable.
		// The solution to solve this is by limiting only the general acceleration and
		// not angular acceleration. To do that we have to find and limit the general
		// velocity before it was combined with angular velocity. That's what's
		// happening here. 1. Calculate max change in velocity (based on maxAccel, and
		// the time passed since last loop) 2. clamp new velocity inside that range
		double maxDeltaGeneralVelocity = path.maxAcceleration * dt;

		targetVelocity = clamp(targetVelocity, prevGeneralVel - maxDeltaGeneralVelocity,
				prevGeneralVel + maxDeltaGeneralVelocity);
		prevGeneralVel = targetVelocity;

		double leftVel = targetVelocity * (2 + curvature * trackWidth) / 2;
		double rightVel = targetVelocity * (2 - curvature * trackWidth) / 2;

		if (reversed) {
			double newleftVel = -rightVel;
			double newrightVel = -leftVel;

			rightVel = newrightVel;
			leftVel = newleftVel;
		}

		log("motoroutput", leftVel + "; " + rightVel);

		calculateFinishedPath(robotPos);
		log("finishedpath", done ? 1 : 0);

		log("pathfollowcalctime", execTimer.time());

		if (done) {
			return new DriveMotorState(0, 0);
		}
		return new DriveMotorState(leftVel, rightVel);
	}

	private void calculateFinishedPath(Point robotPos) {
		int currentPathIndex = getClosestWaypointIndex(robotPos);
		if ((currentPathIndex + 0.0) / (path.waypoints.size() + 0.0) < 0.5) {
			// not even half done, haven't finished path
			// done = false;
			return;
		}
		log("progress", (currentPathIndex + 0.0) / (path.waypoints.size() + 0.0));
		log("currentIndex", currentPathIndex);
		log("totalIndex", path.waypoints.size());
		double distance = distanceBetween(robotPos, path.waypoints.get(path.waypoints.size() - 1).p);
		if (distance <= targetTolerance) {
			log("We are pretty close to target, let's see if we go even closer.");
			if (distance > prevDist2Target) {
				log("Ok, we are starting to get further away from the target, let's stop now.");
				done = true;
				log("FINISHED PATH!!!!!!!!!!!!");
			} else {
				log("Ok, we are going to drive a tiny bit further to see if we can still get closer.");
			}
			prevDist2Target = distance;
		}
	}

	/**
	 * This function gets the closest point to the robot that is on the path.
	 * However, it doesn't just simply returns a known point, in fact, it
	 * interpolates the points if the closest point is in between two known points.
	 * 
	 * This function also updates the "onPath" variable which tells the rest of the
	 * program if the robot is on path or not. If the lookahead point is able to
	 * reach the path (distance between robot and closet point < lookahead
	 * distance), it is on path, otherwise not.
	 * 
	 * @param robotPos robotPosition (wrapped in a {@link Point}) is needed to do
	 *                 the caluclations
	 * 
	 * @return the interpolated waypoint (wrapped in a {@link Waypoint})
	 */
	private Waypoint getInterpolatedWaypoint(Point robotPos) {

		// get the two closest points on path to robot
		int index1 = getClosestWaypointIndex(robotPos);
		int index2 = findClosestWaypointNeighbor(index1, robotPos);

		// get the distance from robot to those two points (index1, index2)
		double d1 = distanceBetween(robotPos, path.waypoints.get(index1).p);
		double d2 = distanceBetween(robotPos, path.waypoints.get(index2).p);

		// a vector representing the distance between those two points (index1, index2)
		Vector deltaVector = new Vector(path.waypoints.get(index1).p, path.waypoints.get(index2).p);

		// scaler distance between those two points
		double d = deltaVector.length();

		// the scaler distance from index1 -> closest point (which is the intercept of
		// the path line and the perpendicular line of the path which also goes through
		// the robotPos) (see diagram: marked "x")
		double scale = (d * d + d1 * d1 - d2 * d2) / (2 * d);

		// this is "x" with direction AND magnitude
		Vector xVector = deltaVector.scale(scale / d);

		// because x has the same ratio of robotPos (relative to index1 and index2),
		// we now only need to find "x"'s ratio; this is eaiser because "x" is on the
		// path line
		double ratio = xVector.length() / d;

		// the purpose of all of this is to find the velocity in between two known
		// waypoints (index1, index2), and this is done linearly.
		// therefore: velocity = base velocity + deltaVelocity
		// velocity = idx1.v + v.diff * ratio (diff = idx2.v-idx1.v)
		double vel = path.waypoints.get(index1).v
				+ ratio * (path.waypoints.get(index1).v - path.waypoints.get(index2).v);

		// finally put all that info into a new waypoint
		// the position of that point = base position + deltaPosition
		// so position = idx1.position + xVector
		Waypoint newWaypoint = new Waypoint(
				new Point(path.waypoints.get(index1).p.x + xVector.dx, path.waypoints.get(index1).p.y + xVector.dy),
				vel);

		// // uh oh, the point is actually off our segment. because this is calculated
		// // based on the formula of this segment, it assumes this segment extends to
		// // infinity, however, we need to check if the perpendicular line actually
		// lands
		// // on our limited segment
		// if (ratio > 1) {
		// // there's nothing we can do now, so lets return the starting point of the
		// path
		// newWaypoint = path.waypoints.get(0).copy();
		// }
		// this is the distance from robotPos to closest point on path
		Vector distance = new Vector(robotPos, newWaypoint.p);
		if (distance.length() < lookAheadDistance) {
			onPath = true;
		} else {
			onPath = false;
		}
		log("onpath", onPath ? 1 : 0);

		closestpoint = newWaypoint;
		return newWaypoint;
	}

	public double getCurvature(Point robotPos, double gyro) {// curvatureOfArcFromRobotToLookAheadPoint

		gyro = Math.PI - gyro;

		Point lookAheadPoint = lookAheadPoint(robotPos);
		log("robotpos", robotPos);
		log("lookaheadpoint", lookAheadPoint);
		double horizontalDistance2LookAheadPoint;
		double a = -Math.tan(gyro);
		double b = 1;
		double c = Math.tan(gyro) * robotPos.x - robotPos.y;

		horizontalDistance2LookAheadPoint = Math.abs(a * lookAheadPoint.x + b * lookAheadPoint.y + c)
				/ (Math.sqrt(a * a + b * b));
		log("horizontalDistance2LookAheadPoint: " + horizontalDistance2LookAheadPoint);

		double side = Math.signum(
				Math.sin(gyro) * (lookAheadPoint.x - robotPos.x) - Math.cos(gyro) * (lookAheadPoint.y - robotPos.y));

		// see if point b is above or under r
		// side *= Math.signum(Math.sin(gyro));

		log("side: " + side);

		double curvature = 2 * side * horizontalDistance2LookAheadPoint / lookAheadDistance / lookAheadDistance;

		log("curvature", curvature);
		return curvature;
	}

	private int getClosestWaypointIndex(Point robotPos) {
		// log(robotPos);
		double minDistanceSoFar = distanceBetween(path.waypoints.get(0).p, robotPos); // set the min distance to
																						// "to-path-starting-point"
																						// distance
		int minDistanceIndexSoFar = 0;

		// don't look back, and also don't look too far ahead (avoids losing path at
		// cross section)
		int searchFrom = Math.max(lastClosestPointPathIndex - 1, 0);
		int searchTo = Math.min(searchFrom + 5, path.waypoints.size()); // path.waypoints.size() - 1;//
																		// Math.min(lastClosestPointPathIndex +
																		// searchLimit,
																		// path.waypoints.size());

		for (int i = searchFrom; i < searchTo; i++) {
			double distance = distanceBetween(path.waypoints.get(i).p, robotPos);
			if (distance < minDistanceSoFar) {
				minDistanceSoFar = distance;
				minDistanceIndexSoFar = i;
			}
		}
		lastClosestPointPathIndex = minDistanceIndexSoFar;
		// log("Found closest point to robot on path: index=" + minDistanceIndexSoFar +
		// ", distance=" + minDistanceSoFar);
		return minDistanceIndexSoFar;
	}

	private Point lookAheadPoint(Point robotPos) {

		int searchFrom;
		int searchTo;
		double mLookAheadDistance;
		if (onPath) {
			// only search from the index where we last left off, also only search to
			// searchLiimit; but all of this is under the constraint of the path length (we
			// don't want a array out of bounds exception)
			// searchFrom = Math.max(lastLookAheadPointPathIndex - 1, 0);
			// searchTo = Math.min(lastLookAheadPointPathIndex + searchLimit,
			// path.waypoints.size() - 1);
			searchFrom = Math.max(lastLookAheadPointPathIndex, 0);// Math.max(getClosestWaypointIndex(robotPos), 0);
			searchTo = path.waypoints.size() - 1;
			mLookAheadDistance = lookAheadDistance;

			for (int i = searchFrom; i < searchTo; i++) {
				Point byRefPoint = new Point();
				if (lineCircleInterception(path.waypoints.get(i).p, path.waypoints.get(i + 1).p, byRefPoint, robotPos,
						mLookAheadDistance)) {
					// this function returns true or false signaling success or fail,
					// the point is passed back by reference
					lastLookAheadPointPathIndex = i;
					// log("FOUND intercept at [" + i + "]", MessageLevel.Info);
					// log("lookAheadPoint:");
					// log(byRefPoint);
					prevLookAheadPoint = byRefPoint.copy();
					return byRefPoint;
				} else { // DO NOT USE POINT AS INTERCEPTION FAILED
					// log("Circle and current line segment[" + i + "] do not intercept, trying next
					// segment.",
					// MessageLevel.Info);
				}
			}
		} else { // ONPATH==FALSE if not on path, perform full search
			searchFrom = path.waypoints.size() - 1 - 1;// index is size -1, but we want index-1
			searchTo = 0;
			mLookAheadDistance = lookAheadDistance * 2;

			for (int i = searchFrom; i > searchTo; i--) { // caution: it is < instead of > here and also -- instead of
															// ++ because we are reverse looping
				Point byRefPoint = new Point();
				if (lineCircleInterception(path.waypoints.get(i).p, path.waypoints.get(i + 1).p, byRefPoint, robotPos,
						mLookAheadDistance)) {
					// this function returns true or false signaling success or fail,
					// the point is passed back by reference
					lastLookAheadPointPathIndex = i;
					// log("FOUND intercept at [" + i + "]", MessageLevel.Info);
					// log("lookAheadPoint:");
					// log(byRefPoint);
					prevLookAheadPoint = byRefPoint.copy();
					return byRefPoint;
				} else { // DO NOT USE POINT AS INTERCEPTION FAILED
					// log("Circle and current line segment[" + i + "] do not intercept, trying next
					// segment.",
					// MessageLevel.Info);
				}
			}
		}
		log("Lookahead Search Range: " + searchFrom + "- " + searchTo);

		log("Circle and ALL line segments do not intercept; robot has lost its path; returning last closest point or last lookahead point for it to get back.");

		if (prevLookAheadPoint != null) {
			return prevLookAheadPoint;
		} else {
			log("prevLookAheadPoint == null; using closest point");
			int index1 = getClosestWaypointIndex(robotPos);
			int index2 = findClosestWaypointNeighbor(index1, robotPos);
			return path.waypoints.get(Math.max(index1, index2)).p;
		}
	}

	private boolean lineCircleInterception(Point startOfLine, Point endOfLine, Point byRefReturnPoint, Point robotPos,
			double radius) {

		Vector d = new Vector(endOfLine.subtract(startOfLine));
		Vector f = new Vector(startOfLine.subtract(robotPos));

		double a = d.dotProduct(d);
		double b = 2 * f.dotProduct(d);
		double c = f.dotProduct(f) - radius * radius;
		boolean t1Valid = false;
		boolean t2Valid = false;
		double discriminant = b * b - 4 * a * c;

		if (discriminant >= 0) {
			discriminant = Math.sqrt(discriminant);
			double t1 = (-b + discriminant) / (2 * a);
			double t2 = (-b - discriminant) / (2 * a);
			if (t1 >= 0 && t1 <= 1) {
				t1Valid = true;
			}
			if (t2 >= 0 && t2 <= 1) {
				t2Valid = true;
			}
			// log("t1 & t2 =" + round(t1, 2) + " " + round(t2, 2));
			if (t1Valid && t2Valid) {
				// log("2 solutions");
				Vector v1 = d.scale(t1);
				Point p1 = new Point(startOfLine.x + v1.dx, startOfLine.y + v1.dy);
				Vector v2 = d.scale(t2);
				Point p2 = new Point(startOfLine.x + v2.dx, startOfLine.y + v2.dy);
				if (firstHasGreaterIndex(p1, p2)) {
					byRefReturnPoint.set(p1);
					log("LookaheadPoint p1 index > p2 index");
				} else {
					byRefReturnPoint.set(p2);
					log("LookaheadPoint p2 index > p1 index");

				}
			} else if (t1Valid && t2Valid == false) {
				// log("solution t1");
				Vector v1 = d.scale(t1);
				Point p1 = new Point(startOfLine.x + v1.dx, startOfLine.y + v1.dy);
				byRefReturnPoint.set(p1);
			} else if (t1Valid == false && t2Valid) {
				// log("solution t2");
				Vector v2 = d.scale(t2);
				Point p2 = new Point(startOfLine.x + v2.dx, startOfLine.y + v2.dy);
				byRefReturnPoint.set(p2);
			} else if (t1Valid == false && t2Valid == false) {
				// log("t1 t2 not in range");
				byRefReturnPoint.set(0, 0);
				return false;
			}
			return true;
		} else {
			// log("delta <0");
			byRefReturnPoint.set(0, 0);
			return false;
		}
	}

	public static double round(double value, int places) {
		// return value;
		if (places < 0)
			throw new IllegalArgumentException();

		BigDecimal bd = new BigDecimal(value);
		bd = bd.setScale(places, RoundingMode.HALF_UP);
		return bd.doubleValue();
	}

	private boolean firstHasGreaterIndex(Point p1, Point p2) {
		int index1 = getClosestWaypointIndex(p1);
		int index2 = findClosestWaypointNeighbor(index1, p1);

		// log("index1= " + Math.max(index11, index12) + " index2= " + Math.max(index21,
		// index22));

		// ok, this is the point in front of both points, lets see which is closer to
		// that point
		int index = Math.max(index1, index2);

		// if p1, is closer to the point in front, then p1 is is more in front
		if (distanceBetween(p1, path.waypoints.get(index).p) <= distanceBetween(p2, path.waypoints.get(index).p)) {
			return true;
		} else {
			return false;
		}
	}

	private double distanceBetween(Point a, Point b) {
		return Math.sqrt(Math.pow(b.x - a.x, 2) + Math.pow(b.y - a.y, 2));
	}

	private int findClosestWaypointNeighbor(int index, Point robotPos) {
		if (index == 0)
			return index + 1;
		else if (index == path.waypoints.size() - 1)
			return index - 1;

		if ((distanceBetween(robotPos, path.waypoints.get(index - 1).p)) < (distanceBetween(robotPos,
				path.waypoints.get(index + 1).p))) {
			return index - 1;
		}
		return index + 1;
	}

	private double getAcceleration(double vel, double prevVel, double dt) {
		return (vel - prevVel) / dt;
	}

	private double clamp(double x, double min, double max) {
		if (x < min)
			x = min;
		else if (x > max)
			x = max;
		return x;
	}

	// private void log(Object o) {
	// System.out.println(o);
	// }

	// private void log(Object o, MessageLevel l) {
	// System.out.println(l.name().toUpperCase() + " " + o);
	// }
}