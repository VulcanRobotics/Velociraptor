/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1218.robot;

import java.io.FileInputStream;
import java.util.Properties;
import java.util.stream.Stream;

import org.team1218.lib.PropertiesManager;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	public static int[] leftMotorControllerIds;  //{1,0,2};
	public static int[] rightMotorControllerIds;  //{14,13,15};
	public static boolean leftInverted;			// false;
	public static boolean rightInverted;		// true;

	public static double[] leftLowGearPIDF, leftHighGearPIDF, rightLowGearPIDF, rightHighGearPIDF;
	public static int lowGearMaxSpeed, highGearMaxSpeed, encTicksPerRev;
	public static double trackWidthInches;
	
	public static int shifterPort, ptoPort, intakePort, armPort;
	
	public static int[] intakeMotorIds;				// {4,11};
	public static boolean[] intakeMotorInvert = new boolean[2]; 		// {true,false};
	public static int[] elevatorMotorIds;			//{3,12};
	public static boolean elevatorMotorInvert; 		// true;
	public static double[] elevatorPIDF;
	public static int elevatorCruiseVelocity, elevatorAcceleration;
	
	public static void loadProperties() {
		PropertiesManager pm = new PropertiesManager("/home/lvuser/robot.properties");
		System.out.println("RobotMap: loading properties");
		pm.load();		

		leftMotorControllerIds = pm.getInts("leftMotorControllerIds", new int[] {0,1,2});
		rightMotorControllerIds = pm.getInts("rightMotorControllerIds",new int[] {14,13,15});
		leftInverted = pm.getBoolean("leftDriveInverted",false);
		rightInverted = pm.getBoolean("rightDriveInverted",true);
		leftLowGearPIDF = pm.getDoubles("leftLowGearPIDF",new double[] {1.1,0.0,10,0.79});
		rightLowGearPIDF = pm.getDoubles("rightLowGearPIDF", new double[] {1.05,0,50,0.79});
		leftHighGearPIDF = pm.getDoubles("leftHighGearPIDF", new double[] {0.0,0.0,0.0,0.0});
		rightHighGearPIDF = pm.getDoubles("rightHighGearPIDF", new double[] {0.0,0.0,0.0,0.0});;
		lowGearMaxSpeed = pm.getInt("lowGearMaxSpeed");
		highGearMaxSpeed = pm.getInt("highGearMaxSpeed");
		encTicksPerRev = pm.getInt("encTicksPerRev");
		trackWidthInches = pm.getDouble("trackWidthInches");
			
		intakeMotorIds = pm.getInts("intakeMotorIds", new int[] {4,11});
		intakeMotorInvert = pm.getBooleans("intakeMotorInvert");
			
		elevatorMotorIds = pm.getInts("elevatorMotorIds", new int[] {3,12});
		elevatorMotorInvert = pm.getBoolean("elevatorMotorInvert",false);
		elevatorPIDF = pm.getDoubles("elevatorPIDF", new double[] {0.0,0.0,0.0,0.0});			
		elevatorCruiseVelocity = pm.getInt("elevatorCruiseVelocity");
		elevatorAcceleration = pm.getInt("elevatorAcceleration");
			
		shifterPort = pm.getInt("shifterPort");
		ptoPort = pm.getInt("ptoPort");
		armPort = pm.getInt("armPort");
		intakePort = pm.getInt("intakePort");
		System.out.println("RobotMap: properties loaded!");
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
