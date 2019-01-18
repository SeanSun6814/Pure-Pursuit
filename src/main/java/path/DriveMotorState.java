package path;

public class DriveMotorState {
	public double leftVel, leftAcc;
	public double rightVel, rightAcc;

	public DriveMotorState(double leftVel, double rightVel) {
		this.leftVel = leftVel;
		this.rightVel = rightVel;
	}
}
