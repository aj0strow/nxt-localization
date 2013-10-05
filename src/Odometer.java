/*
 * File: Odometer.java
 * Written by: Sean Lawlor
 * ECSE 211 - Design Principles and Methods, Head TA
 * Fall 2011
 * 
 * Class which controls the odometer for the robot
 * 
 * Odometer defines cooridinate system as such...
 * 
 * 					90Deg:pos y-axis
 * 							|
 * 							|
 * 							|
 * 							|
 * 180Deg:neg x-axis------------------0Deg:pos x-axis
 * 							|
 * 							|
 * 							|
 * 							|
 * 					270Deg:neg y-axis
 * 
 * The odometer is initalized to 90 degrees, assuming the robot is facing up the positive y-axis
 * 
 */

import lejos.util.Timer;
import lejos.util.TimerListener;

public class Odometer implements TimerListener {
	private static final int PERIOD = 20;
	private Timer timer;
	
	private TwoWheeledRobot robot;
	
	Position position;
	
	private double[] oldDH, dDH;
	
	private Object lock;
	
	// constructor
	public Odometer (TwoWheeledRobot robot) {
		this.robot = robot;
		
		lock = new Object();
		
		setPosition( new Position(0.0, 0.0, Math.PI / 2) );
		
		this.oldDH = new double[2];
		this.dDH = new double[2];
		
		(new Timer(PERIOD, this)).start();
	}
	
	/*
	 * Recompute the odometer values using the displacement and heading changes
	 */
	public void timedOut() {
		robot.getDisplacementAndHeading(dDH);
		incrPosition(dDH[0] - oldDH[0], dDH[1] - oldDH[1]);
		
		oldDH[0] = dDH[0];
		oldDH[1] = dDH[1];
	}
	
	// get, set, incr position
	
	public Position getPosition() {
		Position position = null;
		synchronized (lock) {
			position = this.position.clone();
		}
		return position;
	}
	
	public void setPosition(Position position) {
		synchronized (lock) {
			this.position = position.clone();
		}
	}
	
	public void incrPosition(double dDistance, double dHeading) {
		double theta = getTheta();
		
		double dx = dDistance * Math.cos(theta);
		double dy = dDistance * Math.sin(theta);
		
		incrPosition(dx, dy, dHeading);
	}
	
	public void incrPosition(double dx, double dy, double dtheta) {
		synchronized (lock) {
			this.position.x += dx;
			this.position.y += dy;
			this.position.theta = Angle.normalize(this.position.theta + dtheta);
		}
	}
	
	// 0 <= theta < 2π
	public double getTheta() {
		double result;
		synchronized (lock) { result = position.theta; }
		return result;
	}
	
	public TwoWheeledRobot getRobot() {
		return this.robot;
	}
}
