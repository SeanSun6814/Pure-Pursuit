package path;

public class DriveMotorState {
	public double leftVel, leftAcc;
	public double rightVel, rightAcc;

	public DriveMotorState(double leftVel, double leftAcc, double rightVel, double rightAcc) {
		this.leftVel = leftVel;
		this.leftAcc = leftAcc;
		this.rightVel = rightVel;
		this.rightAcc = rightAcc;
	}
}
