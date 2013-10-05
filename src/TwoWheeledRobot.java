import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.LCD;

public class TwoWheeledRobot {
	public static final double DEFAULT_LEFT_RADIUS = 2.75;
	public static final double DEFAULT_RIGHT_RADIUS = 2.75;
	public static final double DEFAULT_WIDTH = 15.8;
	
	public NXTRegulatedMotor leftMotor, rightMotor;
	
	private double leftRadius, rightRadius, width;
	private double forwardSpeed = 0.0, rotationSpeed = 0.0;
	
	public TwoWheeledRobot(NXTRegulatedMotor leftMotor, NXTRegulatedMotor rightMotor,
			double width, double leftRadius, double rightRadius) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.leftRadius = leftRadius;
		this.rightRadius = rightRadius;
		this.width = width;
	}
	
	public TwoWheeledRobot(NXTRegulatedMotor leftMotor, NXTRegulatedMotor rightMotor) {
		this(leftMotor, rightMotor, DEFAULT_WIDTH, DEFAULT_LEFT_RADIUS, DEFAULT_RIGHT_RADIUS);
	}
	
	// accessors
	public double getDisplacement() {
		return (leftMotor.getTachoCount() * leftRadius +
				rightMotor.getTachoCount() * rightRadius) *
				Math.PI / 360.0;
	}
	
	public double getHeading() {
		return (leftMotor.getTachoCount() * leftRadius -
				rightMotor.getTachoCount() * rightRadius) / width;
	}
	
	// [ cm, radians ]
	public void getDisplacementAndHeading(double[] data) {
		int leftTacho, rightTacho;
		leftTacho = leftMotor.getTachoCount();
		rightTacho = rightMotor.getTachoCount();
		
		data[0] = (leftTacho * leftRadius + rightTacho * rightRadius) * Math.PI / 360.0;
		data[1] = Math.toRadians((-leftTacho * leftRadius + rightTacho * rightRadius) / width);
	}

	// float both motors
	public void setFloat() {
		this.leftMotor.stop();
		this.rightMotor.stop();
		this.leftMotor.flt(true);
		this.rightMotor.flt(true);
	}
	
	public void setForwardSpeed(double speed) {
		forwardSpeed = speed;
		setSpeeds(forwardSpeed, rotationSpeed);
	}
	
	public void setRotationSpeed(double speed) {
		rotationSpeed = speed;
		setSpeeds(forwardSpeed, rotationSpeed);
	}
	
	public void setSpeeds(double forwardSpeed, double rotationalSpeed) {
		double leftSpeed, rightSpeed;

		this.forwardSpeed = forwardSpeed;
		this.rotationSpeed = rotationalSpeed;
		
		double leftVelocity = forwardSpeed - rotationalSpeed * width / 2.0;
		double rightVelocity = forwardSpeed + rotationalSpeed * width / 2.0;

		setMotorSpeed(leftMotor, leftVelocity / leftRadius);
		setMotorSpeed(rightMotor, rightVelocity / rightRadius);
	}
	
	public void incrSpeeds(double dForward, double dRotation) {
		setSpeeds(forwardSpeed + dForward, rotationSpeed + dRotation);
	}
		
	private static void setMotorSpeed(NXTRegulatedMotor motor, double velocity) {
		int speed = (int) Math.abs(Math.toDegrees(velocity));
		if (velocity >= 0) {
			motor.forward();
		} else {
			motor.backward();
		}
		motor.setSpeed(Math.min(200, speed));
	}
	
	public void setAcceleration(int acceleration) {
		leftMotor.setAcceleration(acceleration);
		rightMotor.setAcceleration(acceleration);
	}
	
	public NXTRegulatedMotor[] getMotors() {
		return new NXTRegulatedMotor[]{ leftMotor, rightMotor };
	}
}
