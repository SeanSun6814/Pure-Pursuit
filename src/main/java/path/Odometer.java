package path;

/**
 * This is the magical class that estimates the robot's coordinate on an
 * arbitary field. It does so by using geometry to add up little deltas of x and
 * y calculated from encoders and gyros.
 */

public class Odometer {
	private double accumulativeDistance;
	private double x;
	private double y;

	private double prevLeftEncoderValue;
	private double prevRightEncoderValue;
	private static Odometer instance;

	public static Odometer getInstance() {
		if (instance == null)
			instance = new Odometer();
		return instance;
	}

	public Odometer() {
		// setLogSenderName("Odometer");
		reset();
		// log("Started");
		// System.out.println("Odometer started");

	}

	/**
	 * Update the coordinate using the current left and right encoder readings
	 * 
	 * @param leftEncoder  current left encoder distance reading
	 * @param rightEncoder current right encoder distance reading
	 * @param gyroAngle    current gyro angle reading in degrees (front is 0 degs,
	 *                     clockwise is positive)
	 */
	public void update(double leftEncoder, double rightEncoder, double gyroAngle) {
		gyroAngle = Math.toRadians(gyroAngle + 90);

		gyroAngle = Math.PI - gyroAngle;

		double deltaLeftEncoder = leftEncoder - prevLeftEncoderValue;
		double deltaRightEncoder = rightEncoder - prevRightEncoderValue;
		double distance = (deltaLeftEncoder + deltaRightEncoder) / 2;

		x += distance * Math.cos(gyroAngle);
		y += distance * Math.sin(gyroAngle);

		accumulativeDistance += Math.abs(distance);
		prevLeftEncoderValue = leftEncoder;
		prevRightEncoderValue = rightEncoder;
	}

	/** Reset the odometer to (0, 0) */
	public void reset() {
		accumulativeDistance = x = y = prevLeftEncoderValue = prevRightEncoderValue = 0;
		// log("reset");
	}

	/**
	 * Gets the total distance the robot has gone (forwards + backwards)
	 * 
	 * @return the distance in the same units the encoder values were passed in with
	 */
	public double getAccumulativeDistance() {
		return accumulativeDistance;
	}

	/**
	 * 
	 * @return the latest calculated x coordinate in the same units the encoder
	 *         values were passed in with
	 */
	public double getX() {
		return x;
	}

	/**
	 * 
	 * @return the latest calculated y coordinate in the same units the encoder
	 *         values were passed in with
	 */
	public double getY() {
		return y;
	}

	/**
	 * Change the odometer x coordinate to this value (in the same units the encoder
	 * values were passed in with)
	 * 
	 * @param x the desired x coordinate the odometer will correct to
	 */
	public void setX(double x) {
		this.x = x;
	}

	/**
	 * Change the odometer y coordinate to this value (in the same units the encoder
	 * values were passed in with)
	 * 
	 * @param y the desired y coordinate the odometer will correct to
	 */
	public void setY(double y) {
		this.y = y;
	}

	/**
	 * Get the latest coordinate wrapped in a Point class {@link Point}
	 * 
	 * @return a NEWly created point that holds the coordinate (x, y)
	 */
	public Point getPoint() {
		return new Point(x, y);
	}

	/**
	 * Get the coordinate and the accumulative distance written nicely in one line
	 * 
	 * @return a string with these values separated in semi-colons so it does not
	 *         interfere with comma separated files
	 */
	@Override
	public String toString() {
		return x + "; " + y + "; " + accumulativeDistance;
	}

}
