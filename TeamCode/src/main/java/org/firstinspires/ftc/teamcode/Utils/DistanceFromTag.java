package org.firstinspires.ftc.teamcode.Utils;

public class DistanceFromTag {
	int tag;
	double distance;

	public DistanceFromTag(int tag, double distance) {
		this.tag = tag;
		this.distance = distance;
	}

	public int getTag() {
		return tag;
	}

	public double getDistance() {
		return distance;
	}

	public void setDistance(double distance) {
		this.distance = distance;
	}

	public void setId(int id) {
		this.tag = id;
	}
}