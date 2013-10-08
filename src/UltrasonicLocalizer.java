import lejos.util.Timer;
import lejos.util.TimerListener;
import lejos.nxt.LCD;
import lejos.nxt.SensorPort;

public class UltrasonicLocalizer implements TimerListener {
	private static final int PERIOD = 20;
		
	public enum LocalizationType { FALLING_EDGE, RISING_EDGE };
	public static double ROTATION_SPEED = Math.PI / 4;
	
	private Timer timer;
	
	private UltrasonicPoller poller;
	
	private Odometer odometer;
	private TwoWheeledRobot robot;
	// private LocalizationType locType;
	
	private boolean started;
	
	// for falling edge / rising edge
	private double startAngle = Double.NaN, stopAngle = Double.NaN;
	
	// for angle position of walls
	private double firstAngle = Double.NaN, secondAngle = Double.NaN;
		
	public UltrasonicLocalizer(Odometer odometer, SensorPort sensorPort) {
		this.odometer = odometer;
		this.robot = odometer.getRobot();
		
		this.poller = new UltrasonicPoller(sensorPort);
		
		this.timer = new Timer(PERIOD, this);
	}
	
	public void timedOut() {
		fallingEdge();
	}
	
	private void fallingEdge() {
		int distance = poller.getDistance();
		if (started) {
			if (Double.isNaN(startAngle)) {
				if (distance < 35) startAngle = odometer.getTheta(); 
			} else {
				if (distance < 25) {
					stopAngle = odometer.getTheta();
					
					double midAngle =  Angle.between(startAngle, stopAngle);
					
					LCD.clear();
					LCD.drawString("high: " + Math.toDegrees(startAngle), 0, 0);
					LCD.drawString("low:  " + Math.toDegrees(stopAngle), 0, 1);
					LCD.drawString("mid:  " + Math.toDegrees(midAngle), 0, 2);
					
					if (Double.isNaN(firstAngle)) {
						this.firstAngle = midAngle;
						resetEdgeAngles();
						robot.setSpeeds(0.0, -ROTATION_SPEED);
					} else {
						robot.setSpeeds(0.0, 0.0);
						this.secondAngle = midAngle;
						
						double cornerAngle = Angle.normalize(secondAngle - Math.abs(secondAngle - firstAngle) / 2);
						double dTheta = Angle.difference(cornerAngle, 5 * Math.PI / 4);
						
						LCD.clear();
						LCD.drawString("1st: " + Math.toDegrees(firstAngle), 0, 0);
						LCD.drawString("2nd: " + Math.toDegrees(secondAngle), 0, 1);
						LCD.drawString("dTh: " + Math.toDegrees(dTheta), 0, 2);
						
						odometer.incrPosition(0.0, 0.0, dTheta);
						timer.stop();
						odometer.displayPosition();
					}
				}
			}			
		} else {
			this.started = distance > 50;
		}
	}
	
	private void resetEdgeAngles() {
		this.startAngle = Double.NaN;
		this.stopAngle = Double.NaN;
		this.started = false;
	}

	public void localize() {
		resetEdgeAngles();
		robot.setRotationSpeed(ROTATION_SPEED);
		timer.start();
	}

}
