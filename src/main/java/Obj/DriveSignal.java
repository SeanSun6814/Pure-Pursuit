package Obj;

public class DriveSignal {
	double l;
	double r;

	public DriveSignal() {
		l = r = 0.0;
	}

	public DriveSignal(double l, double r) {
		this.l = l;
		this.r = r;
	}

	public double getL() {
		return l;
	}

	public void setL(double l) {
		this.l = l;
	}

	public double getR() {
		return r;
	}

	public void setR(double r) {
		this.r = r;
	}

}
