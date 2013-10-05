import lejos.nxt.LCD;
import lejos.util.Timer;
import lejos.util.TimerListener;

public class LCDInfo implements TimerListener{
	public static final int LCD_REFRESH = 100;
	private Odometer odometer;
	
	public LCDInfo(Odometer odometer) {
		this.odometer = odometer;
		
		(new Timer(LCD_REFRESH, this)).start();
	}
	
	public void timedOut() {
		Position position = odometer.getPosition();
		LCD.clear();
		LCD.drawString("X: ", 0, 0);
		LCD.drawString("Y: ", 0, 1);
		LCD.drawString("H: ", 0, 2);
		LCD.drawInt((int) (position.x * 10), 3, 0);
		LCD.drawInt((int) (position.y * 10), 3, 1);
		LCD.drawInt((int) position.theta, 3, 2);
	}
}
