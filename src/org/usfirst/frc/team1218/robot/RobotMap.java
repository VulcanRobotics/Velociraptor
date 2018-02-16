/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1218.robot;

import java.io.FileInputStream;
import java.util.Properties;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	public static int[] leftMotorControllerIds = {1,0,2};
	public static int[] rightMotorControllerIds = {14,13,15};
	public static boolean leftInverted = false;
	public static boolean rightInverted = true;
	public static int shifterPort = 0;
	
	public static int[] intakeMotorIds = {4,11};
	public static int[] elevatorMotorIds = {3,12};
	
	
	public static void loadProperties() {
		Properties props = new Properties();
		try {
			FileInputStream propsFile = new FileInputStream("/home/lvuser/robot.properties");
			props.load(propsFile);
		} catch (Exception e) {}
	}
	
	
	
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
}
