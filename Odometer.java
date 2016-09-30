/*
 * Odometer.java
 */

package ev3Odometer;

public class Odometer extends Thread {
	// robot position
	private double x, y, theta, track, wheelRadius;

	// odometer update period, in ms
	private static final long ODOMETER_PERIOD = 25;

	// lock object for mutual exclusion
	private Object lock;

	// default constructor
	public Odometer(double track, double wheelRadius) {
		x = 0.0;
		y = 0.0;
		theta = 0.0;
		this.track = track;
		this.wheelRadius = wheelRadius;
		
		Lab2.leftMotor.resetTachoCount();
		Lab2.rightMotor.resetTachoCount();
		lock = new Object();
	}

	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;
		long previousTachoL=0;
		long previousTachoR=0;
		long currentTachoL=0;
		long currentTachoR=0;
		
		final double wheelCirc= (Math.PI*wheelRadius)/180;
		
		while (true) {
			updateStart = System.currentTimeMillis();
			// put (some of) your odometer code here

			double leftDistance, rightDistance, displacement, deltaTheta, dX, dY;
			
			// get current tachos
			currentTachoL= Lab2.leftMotor.getTachoCount();
			currentTachoR= Lab2.rightMotor.getTachoCount();
			// calculate change in distace
			leftDistance= wheelCirc*(currentTachoL-previousTachoL);
			rightDistance= wheelCirc*(currentTachoR-previousTachoR);
			// update tachos
			previousTachoL=currentTachoL;
			previousTachoR=currentTachoR;
			
			// calculate change in distance of the robot
			displacement= 0.5*(leftDistance+rightDistance);
			deltaTheta=(leftDistance-rightDistance)/track;
			
			synchronized (lock) {
				// don't use the variables x, y, or theta anywhere but here!
				theta += deltaTheta;
				dX=displacement*Math.sin(theta);
				dY=displacement*Math.cos(theta);
				
				x+=dX;
				y+=dY;
			}

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}

	// accessors
	public void getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = theta;
		}
	}

	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}

	// mutators
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}
}