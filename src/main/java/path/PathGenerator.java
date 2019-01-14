package path;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class PathGenerator {

	public static void main(String[] args) {
		PathGenerator p = new PathGenerator();
		// List<Waypoint> waypoints = Arrays.asList(
		// new Waypoint(new Point(0, 0), 0),
		// new Waypoint(new Point(10, 10), 0),
		// new Waypoint(new Point(20, 30), 0),
		// new Waypoint(new Point(30, 30), 0),
		// new Waypoint(new Point(-10, -10), 0));

		// List<Waypoint> waypoints = Arrays.asList(new Waypoint(new Point(0, 0), 0),
		// new Waypoint(new Point(9, 17), 0),
		// new Waypoint(new Point(19, 60), 0), new Waypoint(new Point(60, 40), 0),
		// new Waypoint(new Point(80, 10), 0));
		List<Waypoint> waypoints = Arrays.asList(new Waypoint(new Point(0, 350), 0),
				new Waypoint(new Point(100, 350), 0), new Waypoint(new Point(150, 300), 0),
				new Waypoint(new Point(150, 200), 0), new Waypoint(new Point(200, 150), 0),
				new Waypoint(new Point(300, 150), 0));

		// Path(spacing, maxVel, kTurnConst
		Path injected = p.generatePath(new Path(10, 100, 10, 3, waypoints));

		// p.trainPolyFit(result, 3);
		// System.out.println("__________===_________");
		System.out.println(injected);
		System.out.println("______________________");

		Path smoothed = p.mSmoother(injected, 0.4, 0.01);
		p.tagVelocity(smoothed);
		System.out.println(smoothed);

	}

	public Path generate(double smooth, double tolerance, Path path) {
		Path injected = generatePath(path);
		System.out.println("beginning smmoth path");
		Path smoothed = mSmoother(injected, smooth, tolerance);
		System.out.println("finished smmoth path");
		tagVelocity(smoothed);
		return smoothed;
	}

	public List<Waypoint> inject(Path path, int numToInject) {

		double[][] orig = waypoints2Array(path.waypoints);

		double morePoints[][];

		// create extended 2 Dimensional array to hold additional points
		morePoints = new double[orig.length + ((numToInject) * (orig.length - 1))][2];

		int index = 0;

		// loop through original array
		for (int i = 0; i < orig.length - 1; i++) {
			// copy first
			morePoints[index][0] = orig[i][0];
			morePoints[index][1] = orig[i][1];
			index++;

			for (int j = 1; j < numToInject + 1; j++) {
				// calculate intermediate x points between j and j+1 original points
				morePoints[index][0] = j * ((orig[i + 1][0] - orig[i][0]) / (numToInject + 1)) + orig[i][0];

				// calculate intermediate y points between j and j+1 original points
				morePoints[index][1] = j * ((orig[i + 1][1] - orig[i][1]) / (numToInject + 1)) + orig[i][1];

				index++;
			}
		}

		// copy last
		morePoints[index][0] = orig[orig.length - 1][0];
		morePoints[index][1] = orig[orig.length - 1][1];
		index++;
		// System.out.println("NumToInject:" + numToInject);
		// System.out.println("More Points: [" + morePoints.length + "]");
		// for (int i = 0; i < morePoints.length; i++) {
		// System.out.println(morePoints[i][0] + ", " + morePoints[i][1]);
		// }
		return array2Waypoints(morePoints);
	}

	public double[][] waypoints2Array(List<Waypoint> waypoints) {
		double[][] arr = new double[waypoints.size()][2];
		for (int i = 0; i < waypoints.size(); i++) {
			arr[i][0] = waypoints.get(i).p.x;
			arr[i][1] = waypoints.get(i).p.y;
		}
		return arr;
	}

	public List<Waypoint> array2Waypoints(double[][] arr) {
		List<Waypoint> waypoints = new ArrayList<Waypoint>();
		for (int i = 0; i < arr.length; i++) {
			waypoints.add(new Waypoint(arr[i][0], arr[i][1]));
		}
		return waypoints;
	}

	/**
	 * Optimization algorithm, which optimizes the data points in path to create a
	 * smooth trajectory. This optimization uses gradient descent. While unlikely,
	 * it is possible for this algorithm to never converge. If this happens, try
	 * increasing the tolerance level.
	 * 
	 * BigO: N^x, where X is the number of of times the while loop iterates before
	 * tolerance is met.
	 * 
	 * @param path
	 * @param weight_data
	 * @param weight_smooth
	 * @param tolerance
	 * @return
	 */
	public List<Waypoint> smoother(Path OrigPath, double weight_data, double weight_smooth, double tolerance) {

		double[][] path = waypoints2Array(OrigPath.waypoints);
		// copy array
		double[][] newPath = waypoints2Array(OrigPath.waypoints);

		double change = tolerance;
		while (change >= tolerance) {
			change = 0.0;
			for (int i = 1; i < path.length - 1; i++)
				for (int j = 0; j < path[i].length; j++) {
					double aux = newPath[i][j];
					newPath[i][j] += weight_data * (path[i][j] - newPath[i][j])
							+ weight_smooth * (newPath[i - 1][j] + newPath[i + 1][j] - (2.0 * newPath[i][j]));
					change += Math.abs(aux - newPath[i][j]);
				}
		}

		return array2Waypoints(newPath);
	}

	//
	// public Path processEachSegment(Path segment) {
	//
	// injectPath(segment, 0);
	// smoother(smoothPath, 0.1, 0.3, 0.0000001);
	//
	// for (int i = 0; i < segment.waypoints.size(); i++) {
	//
	// }
	// totalWaypoints.add(waypoints.get(waypoints.size() - 1));
	//
	// Path returnPath = new Path(path);
	// returnPath.waypoints = totalWaypoints;
	// return returnPath;
	// }
	//
	//
	//
	// public Path injectPath(Path path, int index) {
	//// this function injects (fills in) points into the original path to reach
	// desired accuracy (spacing)
	//
	//
	// List<Waypoint> waypoints = path.waypoints;
	// double spacing = path.spacing;
	//
	// List<Waypoint> totalWaypoints = new ArrayList<Waypoint>();
	//
	// //fill in
	//
	// for (int i = 0; i < waypoints.size() - 1; i++) {
	//
	// //total distance between the two waypoints
	// double distance = distanceBetween(waypoints.get(i).p, waypoints.get(i +
	// 1).p);
	//
	// //this is the interval that we want to achieve (the distance between two
	// injections)
	// Vector normalizedVector = new Vector(waypoints.get(i).p, waypoints.get(i +
	// 1).p).normalize().scale(spacing);
	//
	// //so we need... how many injection points?
	// int numOfWaypoints = (int) Math.floor(distance / spacing) + 1;
	//
	//
	// for (int j = 0; j < numOfWaypoints; j++) {
	// totalWaypoints.add(new Waypoint(new Point(waypoints.get(i).p.x + j *
	// normalizedVector.dx,
	// waypoints.get(i).p.y + j * normalizedVector.dy), vel)); // TODO: velocity?
	// }
	// }
	// totalWaypoints.add(waypoints.get(waypoints.size() - 1));
	//
	// Path returnPath = new Path(path);
	// returnPath.waypoints = totalWaypoints;
	// return returnPath;
	// }

	@Deprecated
	public Path generatePath(Path path) {

		List<Waypoint> waypoints = path.waypoints;
		double spacing = path.spacing;

		List<Waypoint> totalWaypoints = new ArrayList<Waypoint>();

		for (int i = 0; i < waypoints.size() - 1; i++) {
			double distance = distanceBetween(waypoints.get(i).p, waypoints.get(i + 1).p);
			Vector normalizedVector = new Vector(waypoints.get(i).p, waypoints.get(i + 1).p).normalize().scale(spacing);
			int numOfWaypoints = (int) Math.floor(distance / spacing) + 1;
			for (int j = 0; j < numOfWaypoints; j++) {
				double vel = waypoints.get(i).v
						+ j * (spacing / distance) * (waypoints.get(i + 1).v - waypoints.get(i).v);
				// vel at between two points: lowerPoint+ j*(unitstep*difference); step=
				// difference *

				totalWaypoints.add(new Waypoint(new Point(waypoints.get(i).p.x + j * normalizedVector.dx,
						waypoints.get(i).p.y + j * normalizedVector.dy), vel)); // TODO: velocity?
			}
		}
		totalWaypoints.add(waypoints.get(waypoints.size() - 1));

		Path returnPath = new Path(path);
		returnPath.waypoints = totalWaypoints;
		return returnPath;
	}

	@Deprecated
	public Path mSmoother(Path originalPath, double weight_smooth, double tolerance) {

		double weight_data = 1 - weight_smooth;

		List<Waypoint> path = originalPath.waypoints;
		List<Waypoint> newPath = originalPath.copy().waypoints;

		double change = tolerance;
		while (change >= tolerance) {
			change = 0.0;
			for (int i = 1; i < path.size() - 1; i++) {// 1 to size()-1: don't stretch first and last point as they
														// should not move
				double aux = newPath.get(i).p.x;
				newPath.get(i).p.x += weight_data * (path.get(i).p.x - newPath.get(i).p.x) + weight_smooth
						* (newPath.get(i - 1).p.x + newPath.get(i - 1).p.x - (2.0 * newPath.get(i).p.x));

				change += Math.abs(aux - newPath.get(i).p.x);

				aux = newPath.get(i).p.y;
				newPath.get(i).p.y += weight_data * (path.get(i).p.y - newPath.get(i).p.y) + weight_smooth
						* (newPath.get(i - 1).p.y + newPath.get(i - 1).p.y - (2.0 * newPath.get(i).p.y));

				change += Math.abs(aux - newPath.get(i).p.y);

				// double aux = newPath[i][j];
				// newPath[i][j] += weight_data * (path[i][j] - newPath[i][j])
				// + weight_smooth * (newPath[i - 1][j] + newPath[i + 1][j] - (2.0 *
				// newPath[i][j]));
				// change += Math.abs(aux - newPath[i][j]);
			}
		}
		Path returnPath = new Path(originalPath);
		returnPath.waypoints = newPath;
		return returnPath;
	}

	public double getPathDistance(Path path) {
		double distance = 0;
		for (int i = 0; i < path.waypoints.size() - 1; i++) {
			distance += distanceBetween(path.waypoints.get(i).p, path.waypoints.get(i + 1).p);
		}
		return distance;
	}

	public void tagInjectionCounter2Steps(Path nodeOnlyPath) {
		int first = 0;
		int second = 0;
		int third = 0;

		double oldPointsTotal = 0;
		double numFinalPoints = nodeOnlyPath.numFinalPoints;
		numFinalPoints = 0;

		int[] ret = null;

		double totalPoints = getPathDistance(nodeOnlyPath) / nodeOnlyPath.spacing;
		totalPoints = Math.max(totalPoints, 51);
		// somehow under 51, it doesn't want to inject anything
		System.out.println("TotalWantedPoints: " + totalPoints);

		int numNodeOnlyPoints = nodeOnlyPath.waypoints.size();
		if (totalPoints < 100) {
			System.out.println("entered <100");
			double pointsFirst = 0;
			double pointsTotal = 0;

			for (int i = 4; i <= 6; i++) {
				// System.out.println("inside first III");
				for (int j = 1; j <= 8; j++) {
					// System.out.println("inside first JJJ");

					pointsFirst = i * (numNodeOnlyPoints - 1) + numNodeOnlyPoints;
					pointsTotal = (j * (pointsFirst - 1) + pointsFirst);

					if (pointsTotal <= totalPoints && pointsTotal > oldPointsTotal) {

						// System.out.println("Setting i, and j to first and second!: " + i + "," + j);
						first = i;
						second = j;
						numFinalPoints = pointsTotal;
						oldPointsTotal = pointsTotal;
					}
				}
			}

			ret = new int[] { first, second, third };
			// System.out.println("InjectionSteps");
			// for (int i = 0; i < ret.length; i++) {
			// System.out.println(ret[i]);
			// }
		} else {

			double pointsFirst = 0;
			double pointsSecond = 0;
			double pointsTotal = 0;

			for (int i = 1; i <= 5; i++)
				for (int j = 1; j <= 8; j++)
					for (int k = 1; k < 8; k++) {
						pointsFirst = i * (numNodeOnlyPoints - 1) + numNodeOnlyPoints;
						pointsSecond = (j * (pointsFirst - 1) + pointsFirst);
						pointsTotal = (k * (pointsSecond - 1) + pointsSecond);

						if (pointsTotal <= totalPoints) {
							first = i;
							second = j;
							third = k;
							numFinalPoints = pointsTotal;
						}
					}

			ret = new int[] { first, second, third };
		}

		nodeOnlyPath.injectionSteps = ret.clone();
		nodeOnlyPath.numFinalPoints = numFinalPoints;
	}

	public Path calculate(Path nodeOnlyPath) {
		double pathAlpha = 0.7, pathBeta = 0.3, pathTolerance = 0.0000001;

		// Figure out how many nodes to inject
		tagInjectionCounter2Steps(nodeOnlyPath);

		Path smoothPath = new Path(nodeOnlyPath);
		smoothPath.waypoints = new ArrayList<Waypoint>();

		smoothPath.waypoints = inject(nodeOnlyPath, nodeOnlyPath.injectionSteps[0]);
		smoothPath.waypoints = smoother(smoothPath, pathAlpha, pathBeta, pathTolerance);
		// iteratively inject and smooth the path
		for (int i = 1; i < nodeOnlyPath.injectionSteps.length; i++) {
			smoothPath.waypoints = inject(smoothPath, nodeOnlyPath.injectionSteps[i]);
			smoothPath.waypoints = smoother(smoothPath, 0.1, 0.3, 0.0000001);
		}
		tagVelocity(smoothPath);
		smoothPath.waypoints.remove(smoothPath.waypoints.size() - 1);
		return smoothPath;
	}

	public void tagVelocity(Path path) {

		List<Waypoint> waypoints = path.waypoints;

		// getting direct DISTANCE between all adjacent smoothed points
		for (int i = 0; i < waypoints.size() - 1; i++) {
			waypoints.get(i).distance = distanceBetween(waypoints.get(i).p, waypoints.get(i + 1).p);
		}

		// getting the CURVATURE of all points (except endpoints) using adjacent points
		for (int i = 1; i < waypoints.size() - 1; i++) {
			double x1 = waypoints.get(i).p.x + 0.0001, y1 = waypoints.get(i).p.y + 0.0001;
			double x2 = waypoints.get(i - 1).p.x, y2 = waypoints.get(i - 1).p.y;
			double x3 = waypoints.get(i + 1).p.x, y3 = waypoints.get(i + 1).p.y;

			double k1 = 0.5 * (x1 * x1 + y1 * y1 - x2 * x2 - y2 * y2) / (x1 - x2);
			double k2 = (y1 - y2) / (x1 - x2);
			double b = 0.5 * (x2 * x2 - 2 * x2 * k1 + y2 * y2 - x3 * x3 + 2 * x3 * k1 - y3 * y3)
					/ (x3 * k2 - y3 + y2 - x2 * k2);
			double a = k1 - k2 * b;
			double r = Math.sqrt((x1 - a) * (x1 - a) + (y1 - b) * (y1 - b));
			double curvature = 1 / r;
			// System.out.println(r);

			waypoints.get(i).curvature = curvature;
		}

		// generating IDEAL VELOCITIES at each point based on its curvature
		for (int i = 0; i < waypoints.size(); i++) {
			// there are two constraints on velocity: maxVel (so vel<>infinity on straight
			// paths) && curvature vel (slow down based on curvature)
			// Take the min of those two to satisfy them both
			// double maxAngularVel = 1;
			waypoints.get(i).v = Math.min(path.maxVelocity, path.maxAngleVel / waypoints.get(i).curvature);
		}
		// stop at the end
		// in fact, stop before the path ends (because lookahead point is ahead)
		waypoints.get(waypoints.size() - 1).v = 0;

		// take into account reality by considering maxAcceleration, slow down ahead of
		// time
		// first turn maxAccel with respect to time into maxAccel with respect to
		// DISTANCE

		// loop from back to front and tweak the velocities (leave the endpoint 0)
		for (int i = waypoints.size() - 1 - 1; i >= 0; i--) {
			double distance = distanceBetween(waypoints.get(i + 1).p, waypoints.get(i).p);
			// again, there are two constraints: the wanted velocity at i,
			// and the max velocity at i to achieve i+1's velocity
			// Take the min of those two to satisfy them both

			waypoints.get(i).v = Math.min(waypoints.get(i).v,
					getMaxAchieveableVelocity(path.maxAcceleration, distance, waypoints.get(i + 1).v));
		}
	}

	private double getMaxAchieveableVelocity(double timeAccel, double distance, double velocity0) {
		return Math.sqrt(velocity0 * velocity0 + 2 * timeAccel * distance);
	}

	// public double[] trainPolyFit(Path path, int degree) {
	//
	// PolynomialCurveFitter polynomialCurveFitter =
	// PolynomialCurveFitter.create(degree);
	//
	// ArrayList<WeightedObservedPoint> weightedObservedPoints = new
	// ArrayList<WeightedObservedPoint>();
	//
	// for (int i = 0; i < path.waypoints.size(); i++) {
	//
	// WeightedObservedPoint weightedObservedPoint = new WeightedObservedPoint(1,
	// path.waypoints.get(i).p.x,
	// path.waypoints.get(i).p.y);
	//
	// weightedObservedPoints.add(weightedObservedPoint);
	//
	// }
	//
	// double[] result = polynomialCurveFitter.fit(weightedObservedPoints);
	//
	// for (int i = 0; i < result.length; i++) {
	// System.out.println(result[i]);
	// }
	//
	// return result;
	//
	// }

	private double distanceBetween(Point a, Point b) {
		return Math.sqrt(Math.pow(b.x - a.x, 2) + Math.pow(b.y - a.y, 2));
	}
}
