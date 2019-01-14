package path;

import java.math.BigDecimal;
import java.math.RoundingMode;

/**
 * This class defines a waypoint. A Waypoint is a point that stores additional
 * information needed for path following. The most important are p (x, y value
 * at point ), and v (pre-generated ideal velocity at that point)
 */

public class Waypoint {
	public Point p; // stores the coordinate of the waypoint (x,y)
	public double v; // stores the velocity at the waypoint
	public double distance; // stores the smoothed direct distance between this waypoint and the last
							// waypoint (not sure if this is needed at all) TODO: next code cleanup
	public double curvature;// stores the curvature of the smoothed path between this waypoint and the last
							// waypoint, (only needed for path generating)

	public Waypoint(Point p, double v) {
		this.p = p;
		this.v = v;
	}

	public Waypoint(double x, double y) {
		this.p = new Point(x, y);
		this.v = 0;
	}

	public Waypoint(double x, double y, double v) {
		this.p = new Point(x, y);
		this.v = v;
	}

	public Waypoint subtract(Waypoint waypoint) {
		return new Waypoint(this.p.subtract(waypoint.p), this.v - waypoint.v);
	}

	public Waypoint add(Waypoint waypoint) {
		return new Waypoint(this.p.add(waypoint.p), this.v + waypoint.v);
	}

	public Waypoint scale(double scale) {
		return new Waypoint(this.p.scale(scale), this.v * scale);
	}

	public Waypoint copy() {
		return new Waypoint(p.copy(), this.v);
	}

	@Override
	public String toString() {
		return p.toString() + " | " + round(v, 2);// + " dist=" + round(distance, 2) + " curv=" + round(curvature, 2);
		// return p.toString() + "| vel=" + round(v, 2) + " dist=" + round(distance, 2)
		// + " curv=" + round(curvature, 2);
	}

	public static double round(double value, int places) {
		// return value;
		if (places < 0)
			throw new IllegalArgumentException();

		BigDecimal bd = new BigDecimal(value);
		bd = bd.setScale(places, RoundingMode.HALF_UP);
		return bd.doubleValue();
	}

}
