package Obj;

public class IntakeState {
	private IntakeMode mode; //enum of {openLoop, autoMaintain}
	private double speed; // [motor power percentage (0~1)]. intake motor speed, should always be positive
	private IntakeDirection direction; //enum of {in, out}
	private boolean autoHold; //true: enable autoHold (true only valid on autoMaintain)

	public IntakeState() {
		this.speed = 0;
		this.mode = IntakeMode.autoMaintain;
	}

	public IntakeState(double speed, IntakeDirection direction, IntakeMode mode) {
		this.speed = speed;
		this.mode = mode;
		this.direction = direction;
		this.autoHold = false;
	}

	public IntakeState(boolean autoHold) {
		this.speed = 0;
		this.mode = IntakeMode.autoMaintain;
		this.direction = IntakeDirection.in;
		this.autoHold = true;
	}

	public IntakeMode getMode() {
		return mode;
	}

	public void setMode(IntakeMode mode) {
		this.mode = mode;
	}

	public double getSpeed() {
		return speed;
	}

	public void setSpeed(double speed) {
		this.speed = speed;
	}

	public IntakeDirection getDirection() {
		return direction;
	}

	public void setDirection(IntakeDirection direction) {
		this.direction = direction;
	}

	public boolean getAutoHold() {
		return autoHold && (mode == IntakeMode.autoMaintain);
	}

	public void setAutoHold(boolean autoHold) {
		this.autoHold = autoHold;
		this.mode = IntakeMode.autoMaintain;
	}
}
