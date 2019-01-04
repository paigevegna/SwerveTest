package org.usfirst.frc.team2875.robot;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.AnalogGyro;

public class SwerveDrive{
	
	//length, width, ports, max_volts all must be determined
		
	public static final double LENGTH = .517525; //m
	public static final double WIDTH = .5699125;
	private static final double MAX_VOLTS = 4.95;
	private static final double DEADZONE = 5;
	private static final double SERVO_SPEED = 1.0;
	private static final double KP = .5;
	
	private AnalogGyro gyro = new AnalogGyro(0); //get real value
	
	public static final int[][] drivePorts = {{0, 2},//updated
											  {6, 4}};
	
	public static final int[][] turnPorts = {{7, 1},
											 {5, 3}};//updated
	
	private Spark[][] power = {{new Spark(drivePorts[0][0]), new Spark(drivePorts[0][1])},
							   {new Spark(drivePorts[1][0]), new Spark(drivePorts[1][1])}};
	
	private Spark[][] turning = {{new Spark(turnPorts[0][0]), new Spark(turnPorts[0][1])},
			   					 {new Spark(turnPorts[1][0]), new Spark(turnPorts[1][1])}};
	
	private int[][][] encPorts = {{{9, 8}, {7, 6}}, //updated
								  {{0, 1}, {2, 3}}};
	
	//private final double[][] initPos
	
	private Encoder[][] encoders = {{new Encoder(encPorts[0][0][0], encPorts[0][0][1]), new Encoder(encPorts[0][1][0], encPorts[0][1][1])},
									{new Encoder(encPorts[1][0][0], encPorts[1][0][1]), new Encoder(encPorts[1][1][0], encPorts[1][1][1])}};
	
	private PIDController[][] pid = {{new PIDController(.05, 0, 0, encoders[0][0], turning[0][0]), new PIDController(.05, 0, 0, encoders[0][1], turning[0][1])},
									 {new PIDController(.05, 0, 0, encoders[1][0], turning[1][0]), new PIDController(.05, 0, 0, encoders[1][1], turning[1][1])}};
	
	
	public SwerveDrive()
	{
		/*for (PIDController[] side: pid)
		{
			for (PIDController controller: side)
				controller.enable();
		}*/
	}
	
	public void reset()
	{
		for (Encoder[] side: encoders)
		{
			for (Encoder encoder: side)
				encoder.reset();
		}
	}
	
	public void fullDrive(double x1, double y1, double x2) {
		x1 = (x1 < DEADZONE) ? 0 : x1;
		x2 = (x2 < DEADZONE) ? 0 : x2;
		y1 = (y1 < DEADZONE) ? 0 : y1;
		double r = Math.sqrt(LENGTH*LENGTH + WIDTH*WIDTH);
		y1 *= -1;
		
		double a = x1 - x2 * (LENGTH / r);
	    double b = x1 + x2 * (LENGTH / r);
	    double c = y1 - x2 * (WIDTH / r);
	    double d = y1 + x2 * (WIDTH / r);
	    
	    double backRightSpeed = Math.sqrt ((a * a) + (d * d));
	    double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
	    double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
	    double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));
	    
	    double backRightAngle = Math.atan2 (a, d) / Math.PI;
	    double backLeftAngle = Math.atan2 (a, c) / Math.PI;
	    double frontRightAngle = Math.atan2 (b, d) / Math.PI;
	    double frontLeftAngle = Math.atan2 (b, c) / Math.PI;
	    
	    for (int i = 0; i < 2; i++) {
	    	for (int j = 0; j < 2; j++) {
	    		pid[i][j].setOutputRange (-1, 1);
	    		pid[i][j].setInputRange(-1, 1);
	    		pid[i][j].setContinuous(true);
	    		pid[i][j].enable ();
	    	}
	    }
	    
	    power[0][0].set(frontLeftSpeed);
	    power[0][1].set(frontRightSpeed);
	    power[1][0].set(backLeftSpeed);
	    power[1][1].set(backRightSpeed);
	    
	    double[][] setpoint = {{frontLeftAngle * (MAX_VOLTS * 0.5) + (MAX_VOLTS * 0.5), frontRightAngle * (MAX_VOLTS * 0.5) + (MAX_VOLTS * 0.5)},
	    					   {backLeftAngle * (MAX_VOLTS * 0.5) + (MAX_VOLTS * 0.5), backRightAngle * (MAX_VOLTS * 0.5) + (MAX_VOLTS * 0.5)}};
	    
	    for (int i = 0; i < 2; i++) {
	    	for (int j = 0; j < 2; j++) {
	    		
	    		if (setpoint[i][j] < 0) {
	    	        setpoint[i][j] = MAX_VOLTS + setpoint[i][j];
	    	    }
	    	    if (setpoint[i][j] > MAX_VOLTS) {
	    	        setpoint[i][j] = setpoint[i][j] - MAX_VOLTS;
	    	    }
	    	    
	    	    pid[i][j].setSetpoint(setpoint[i][j]);
	    	}
	    }
	    
	}
	
	public void transDrive(double xI, double yI) 
	{
		double x = Math.abs(xI) > DEADZONE ? xI : 0;
		double y = Math.abs(yI) > DEADZONE ? yI : 0;
		
		double theta = (180 / Math.PI) * Math.atan(y/x);
		
		double r = Math.sqrt(x*x + y*y) / Math.sqrt(2);
		
		//Debug
		System.out.println("Theta: " + theta + ", R: " + r);
		
		for (Spark[] side: power)
		{
			for (Spark motor: side)
				motor.set(r);
		}
		for (PIDController[] side: pid)
		{
			for (PIDController controller: side)
				controller.setSetpoint(theta);
		}
	}
	
	public void turn(double degree, Spark servo, Encoder enc) {
		System.out.println("other encoder " + enc.getRaw());
		double speed = .5;
		double goal =  degree;
		servo.set(speed);
		double error = goal - enc.getRaw();
		while(Math.abs(error) > DEADZONE) {
			
			if(error < 0) {
				speed *= -1;
			}
			servo.set(speed);
			error = goal - enc.getRaw();
		}
		
		
	}

	public void drive(double x, double y) {
		double dZone = .05;
		x = Math.abs(x) > dZone ? x : 0;
		y = Math.abs(y) > dZone ? y : 0;
		double angle = Math.atan(y / x) * (180 / Math.PI);
		if (x < 0 && y < 0)
			angle += 90;
		else if (x > 0 && y < 0)
			angle += 180;
		else if (x > 0 && y > 0)
			angle += 270;
		
		/*for (int i = 0; i < 2; i++) {
			for (int j = 0; j < 2; j++) {
				turn(angle, turning[i][j], encoders[i][j]);
			}
		}
		for (int i = 0; i < 2; i++) {
			for (int j = 0; j < 2; j++) {
				turn(angle, turning[i][j], encoders[i][j]);
			}
		}*/
		System.out.println("front left: " + encoders[0][0].getFPGAIndex());
		System.out.println("front right: " + encoders[0][1].getFPGAIndex());
		System.out.println("back left: " + encoders[1][0].getFPGAIndex());
		System.out.println("back right: " + encoders[1][1].getFPGAIndex());
	}
}