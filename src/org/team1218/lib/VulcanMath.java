package org.team1218.lib;

public class VulcanMath {
	public static final double differenceOfTwoAngles(double angle1, double angle2) {
		double difference = angle2 - angle1;
		if(difference > 180) {
			difference -= 360;
		}else if(difference < -180) {
			difference += 360;
		}
		return difference;
	}
}
