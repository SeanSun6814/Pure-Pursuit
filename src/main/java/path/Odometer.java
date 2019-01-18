package path;

public class Odometer /* extends LogBase */ {
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

	public void reset() {
		accumulativeDistance = x = y = prevLeftEncoderValue = prevRightEncoderValue = 0;
		// log("reset");
	}

	public double getAccumulativeDistance() {
		return accumulativeDistance;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public void setX(double x) {
		this.x = x;
	}

	public void setY(double y) {
		this.y = y;
	}

	public Point getPoint() {
		return new Point(x, y);
	}

	@Override
	public String toString() {
		return "Odometer: accumulativeDistance=" + accumulativeDistance + ", x=" + x + ", y=" + y;
	}

}
