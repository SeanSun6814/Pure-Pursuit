package path;

import java.util.ArrayList;
import java.util.List;

public class Path {
	public List<Waypoint> waypoints;
	public double spacing;
	public double maxVelocity;
	public double maxAcceleration;
	public double maxAngleVel;
	public double lookAheadDistance;

	//
	public int[] injectionSteps;
	double numFinalPoints = 0;// Don't touch this, it's in 1712's code algorithm

	@Deprecated
	public Path(List<Waypoint> waypoints, double spacing) {
		this.waypoints = waypoints;
		this.spacing = spacing;
	}

	public Path(double spacing, double maxVelocity, double maxAcceleration, double maxAngleVel,
			List<Waypoint> waypoints) {
		this.waypoints = waypoints;
		this.spacing = spacing;
		this.maxVelocity = maxVelocity;
		this.maxAcceleration = maxAcceleration;
		this.maxAngleVel = maxAngleVel;
	}

	public Path(Path path) {
		this.spacing = path.spacing;
		this.maxVelocity = path.maxVelocity;
		this.maxAcceleration = path.maxAcceleration;
		this.maxAngleVel = path.maxAngleVel;
		this.waypoints = new ArrayList<Waypoint>();
		waypoints.forEach((Waypoint waypoint) -> {
			this.waypoints.add(waypoint.copy());
		});
	}

	public Path copy() {
		Path newPath = new Path(spacing, maxVelocity, maxAcceleration, maxAngleVel, new ArrayList<Waypoint>());

		waypoints.forEach((Waypoint waypoint) -> {
			newPath.waypoints.add(waypoint.copy());
		});
		return newPath;
	}

	@Override
	public String toString() {
		String str = "";
		for (int i = 0; i < waypoints.size(); i++) {
			str += waypoints.get(i).toString() + "\n";
		}
		return str;
	}
}
// Trajectory.Config config = new
// Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
// Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
// Waypoint[] points = new Waypoint[] {
// new Waypoint(-4, -1, Pathfinder.d2r(-45)),
// new Waypoint(-2, -2, 0),
// new Waypoint(0, 0, 0)
// };