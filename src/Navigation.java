/*
 * File: Navigation.java
 * Written by: Sean Lawlor
 * ECSE 211 - Design Principles and Methods, Head TA
 * Fall 2011
 * 
 * Movement control class (turnTo, travelTo, flt, localize)
 */
import lejos.util.Timer;
import lejos.util.TimerListener;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.LCD;

public class Navigation implements TimerListener {
	private final static int PERIOD = 1;
	
	// ACCELERATION: wheel angular acceleration (deg / s^2)
	final static int ACCELERATION = 4000;
	final static double ANGLE_ERROR = Math.PI / 1000.0, CM_ERR = 1.0;
	private Odometer odometer;
	private TwoWheeledRobot robot;
		
	private Position position;
	private Point destination;
	private double angle = Double.NaN;
	
	private double previousDistance, previousDifference;

	public Navigation(Odometer odometer) {
		this.odometer = odometer;
		this.robot = odometer.getRobot();
		this.position = odometer.getPosition();
		
		(new Timer(PERIOD, this)).start();
		
		robot.setAcceleration(ACCELERATION);
	}
	
	public void timedOut() {
		this.position = odometer.getPosition();
		
		
		LCD.clear();
		LCD.drawString("x " + position.x, 0, 0);
		LCD.drawString("y " + position.y, 0, 1);
		LCD.drawString("t " + position.theta, 0, 2);
		
		if (isRotating()) {
			if (turned()) stopRotation();
			else rotate();
		} else if (isNavigating()) {
			if (arrived()) stopNavigation();
			else navigate();
		}
	}
	
	public boolean isNavigating() {
		return destination != null;
	}
	
	private void stopNavigation() {
		robot.setSpeeds(0.0, 0.0);
		this.destination = null;
	}
	
	public boolean isRotating() {
		boolean rotating = !Double.isNaN(angle);		
		return rotating;
	}
	
	private void stopRotation() {
		robot.setSpeeds(0.0, 0.0);
		this.angle = Double.NaN;
	}
	
	private boolean turned() {
		double difference = Angle.difference(position.theta, angle);
		
		double absDifference = Math.abs(difference);
		return absDifference < Math.PI / 50.0 && previousDifference <= absDifference;
	}

	private boolean arrived() {
		double distance = position.distanceTo(destination);
		
		boolean atDestination = distance < 1.0;
		boolean distanceWorsened = distance < 10.0 && previousDistance <= distance;
		this.previousDistance = distance;
		
		return atDestination || distanceWorsened;
	}
	
	private void rotate() {
		robot.setRotationSpeed(Angle.direction(position.theta, angle) * (Math.PI / 8));
	}
		
	private void navigate() {
		robot.setSpeeds(5, 0.0);
	}

	/*
	 * TravelTo function which takes as arguments the x and y position in cm Will travel to designated position, while
	 * constantly updating it's heading
	 */
	public void travelTo(Point point) {
		this.angle = position.angleTo(point);
		this.destination = point.clone();
	}
	
	/*
	 * TurnTo function which takes an angle and boolean as arguments The boolean controls whether or not to stop the
	 * motors when the turn is completed
	 */
	public void turnTo(double angle) {
		robot.setForwardSpeed(0.0);
		this.angle = angle;
	}
	
	// Go foward a set distance in cm
	public void goForward(double distance) {
		double theta = Math.toRadians(this.odometer.getTheta());
		Point destination = new Point(Math.cos(theta) * distance, Math.sin(theta) * distance);
		this.travelTo(destination);
	}

}
