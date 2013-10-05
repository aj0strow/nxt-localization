/*
*  Angle is a collection of static helper methods for doing
*  calculations with angles.
*/

public class Angle {
	
	public static double normalize(double radians) {
		// normalize angle so 0 <= angle < 2π
		
	   double normalized = radians % (2 * Math.PI);
	   if (normalized < 0) normalized += 2 * Math.PI;
	   return normalized;
	}
	
	public static double difference(double startAngle, double stopAngle) {
		// actual radians from start angle to stop angle
		
		double left = normalize(stopAngle - startAngle);
		double right = normalize(startAngle - stopAngle);
		return (left < right) ? left : -right;
	}
	
	public static double direction(double startAngle, double stopAngle) {
		double difference = difference(startAngle, stopAngle);
		return difference / Math.abs(difference);
	}
	
	public static double between(double startAngle, double stopAngle) {		
	   return normalize(startAngle + difference(startAngle, stopAngle) / 2);
	}
	
}