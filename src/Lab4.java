import lejos.nxt.*;

public class Lab4 {

	public static void main(String[] args) {
		Button.ESCAPE.addButtonListener(new ExitListener());
		
		Button.waitForAnyPress();
		
		// setup the odometer, display, and ultrasonic and light sensors
		TwoWheeledRobot robot = new TwoWheeledRobot(Motor.A, Motor.B);
		
		Odometer odometer = new Odometer(robot);
		Navigation navigation = new Navigation(odometer);
		
		navigation.turnTo(0.0);
		waitFor(navigation);
		navigation.turnTo(3 * Math.PI / 2);
		waitFor(navigation);
		navigation.turnTo(Math.PI);
		waitFor(navigation);
		
		/*
		Point[] destinations = new Point[]{
			new Point(0.0, 30.0),
			new Point(30.0, 30.0),
			new Point(30.0, 0.0),
			new Point(0.0, 0.0)
		};
		
		for (Point point : destinations) {
			navigation.travelTo(point);
			while (navigation.isNavigating()) {
				try {
					Thread.sleep(1000);
				} catch(InterruptedException e) {}
			}
		}
		*/
		
		// LCDInfo lcd = new LCDInfo(odometer);

		/*

		UltrasonicSensor us = new UltrasonicSensor(SensorPort.S2);
		LightSensor ls = new LightSensor(SensorPort.S1);
		
		// perform the ultrasonic localization
		USLocalizer usl = new USLocalizer(odo, us, USLocalizer.LocalizationType.FALLING_EDGE);
		usl.doLocalization();
		
		// perform the light sensor localization
		LightLocalizer lsl = new LightLocalizer(odo, ls);
		lsl.doLocalization();	
		
		*/
		
		Button.waitForAnyPress();
	}
	
	private static void waitFor(Navigation navigation) {
		// sleep until the operator is done navigating before setting
		// a new destination
		
		while (navigation.isNavigating() || navigation.isRotating()) {
			try {
				Thread.sleep(1000);
			} catch(InterruptedException e) {}
		}
	}
	

}
