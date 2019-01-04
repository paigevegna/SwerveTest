/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2875.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;

@SuppressWarnings("deprecation")
public class Robot extends SampleRobot {
	
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	
	private SwerveDrive dTrain;
	
	private Joystick strafe = new Joystick(0);
	//private Joystick rotate = new Joystick(1);
	
	private SendableChooser<String> m_chooser = new SendableChooser<>();

	/*public Robot() {
		dTrain.setExpiration(0.1);
	}*/

	@Override
	public void robotInit() {
		dTrain = new SwerveDrive();
		dTrain.reset();
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto modes", m_chooser);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 *
	 * <p>If you wanted to run a similar autonomous mode with an IterativeRobot
	 * you would write:
	 *
	 * <blockquote><pre>{@code
	 * Timer timer = new Timer();
	 *
	 * // This function is run once each time the robot enters autonomous mode
	 * public void autonomousInit() {
	 *     timer.reset();
	 *     timer.start();
	 * }
	 *
	 * // This function is called periodically during autonomous
	 * public void autonomousPeriodic() {
	 * // Drive for 2 seconds
	 *     if (timer.get() < 2.0) {
	 *         myRobot.drive(-0.5, 0.0); // drive forwards half speed
	 *     } else if (timer.get() < 5.0) {
	 *         myRobot.drive(-1.0, 0.0); // drive forwards full speed
	 *     } else {
	 *         myRobot.drive(0.0, 0.0); // stop robot
	 *     }
	 * }
	 * }</pre></blockquote>
	 */
	@Override
	public void autonomous() {
		String autoSelected = m_chooser.getSelected();
		// String autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);

		// MotorSafety improves safety when motors are updated in loops
		// but is disabled here because motor updates are not looped in
		// this autonomous mode.
		
		//m_robotDrive.setSafetyEnabled(false);

		switch (autoSelected) {
			case kCustomAuto:
				// Spin at half speed for two seconds
				dTrain.transDrive(0, 0);
				Timer.delay(2.0);

				// Stop robot
				dTrain.transDrive(0, 0);
				break;
			case kDefaultAuto:
			default:
				// Drive forwards for two seconds
				dTrain.transDrive(0, 0);
				Timer.delay(2.0);

				// Stop robot
				dTrain.transDrive(0, 0);
				break;
		}
	}

	
	@Override
	public void operatorControl() {
		//m_robotDrive.setSafetyEnabled(true);
		while (isOperatorControl() && isEnabled()) {
			
			dTrain.drive(strafe.getX(), strafe.getY());
			System.out.println("test");
			// The motors will be updated every 5ms
			Timer.delay(0.005);
		}
		
	}

	/**
	 * Runs during test mode.
	 */
	@Override
	public void test() {
	}
}
