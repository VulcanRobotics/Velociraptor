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

import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.PathGenerator;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.WaypointSequence;

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

	public static boolean useCamera;
	
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
	
	public static Path rightSwitchPath, leftStartleftScalePath;
	
	public static void makePaths() {
		TrajectoryGenerator.Config driveTrainPathConfig = new TrajectoryGenerator.Config();
		driveTrainPathConfig.dt = .1;			// the time in seconds between each generated segment
		driveTrainPathConfig.max_acc = 7.0;		// maximum acceleration for the trajectory, ft/s
		driveTrainPathConfig.max_jerk = 7.0;	// maximum jerk (derivative of acceleration), ft/s
		driveTrainPathConfig.max_vel = 7.0;		// maximum velocity you want the robot to reach for this trajectory, ft/s
		
		
		WaypointSequence ws = new WaypointSequence(10);
		/* right SWitch path with center start */
        ws.addWaypoint(new WaypointSequence.Waypoint(0.0, 0.0, 0.0));
        ws.addWaypoint(new WaypointSequence.Waypoint(2.0, 0.0, 0.0));
        ws.addWaypoint(new WaypointSequence.Waypoint(11.5, -3.0, Math.toRadians(-10.0)));
        rightSwitchPath = PathGenerator.makePath(ws, driveTrainPathConfig, trackWidthInches, "rightSwitch");
        
        ws = new WaypointSequence(10);
        
		/* testing start left, left scale path */
		ws.addWaypoint(new WaypointSequence.Waypoint(0.0,0.0,0.0));
		ws.addWaypoint(new WaypointSequence.Waypoint(14.0,-1.0,0.0));
		ws.addWaypoint(new WaypointSequence.Waypoint(22.0,-2.0,Math.toRadians(-12.0)));
		leftStartleftScalePath = PathGenerator.makePath(ws, driveTrainPathConfig, trackWidthInches, "leftStartleftScale");
	}
	
	public static void loadProperties() {
		Properties props = new Properties();
		System.out.println("RobotMap: loading properties");
		try {
			FileInputStream propsFile = new FileInputStream("/home/lvuser/robot.properties");
			props.load(propsFile);		

			leftMotorControllerIds = Stream.of(props.getProperty("leftMotorControllerIds").split(",")).mapToInt(Integer::parseInt).toArray();
			rightMotorControllerIds = Stream.of(props.getProperty("rightMotorControllerIds").split(",")).mapToInt(Integer::parseInt).toArray();
			leftInverted = Boolean.parseBoolean(props.getProperty("leftDriveInverted"));
			rightInverted = Boolean.parseBoolean(props.getProperty("rightDriveInverted"));
			leftLowGearPIDF = Stream.of(props.getProperty("leftLowGearPIDF").split(",")).mapToDouble(Double::parseDouble).toArray();
			rightLowGearPIDF = Stream.of(props.getProperty("rightLowGearPIDF").split(",")).mapToDouble(Double::parseDouble).toArray();
			leftHighGearPIDF = Stream.of(props.getProperty("leftHighGearPIDF").split(",")).mapToDouble(Double::parseDouble).toArray();
			rightHighGearPIDF = Stream.of(props.getProperty("rightHighGearPIDF").split(",")).mapToDouble(Double::parseDouble).toArray();
			lowGearMaxSpeed = Integer.parseInt(props.getProperty("lowGearMaxSpeed"));
			highGearMaxSpeed = Integer.parseInt(props.getProperty("highGearMaxSpeed"));
			encTicksPerRev = Integer.parseInt(props.getProperty("encTicksPerRev"));
			trackWidthInches = Double.parseDouble(props.getProperty("trackWidthInches"));
			
			useCamera = Boolean.parseBoolean(props.getProperty("useCamera"));
			
			intakeMotorIds = Stream.of(props.getProperty("intakeMotorIds").split(",")).mapToInt(Integer::parseInt).toArray();
			int i = 0;
			for (String s : props.getProperty("intakeMotorInvert").split(",")) {
				intakeMotorInvert[i] = Boolean.parseBoolean(s);
				i++;
			}
			
			elevatorMotorIds = Stream.of(props.getProperty("elevatorMotorIds").split(",")).mapToInt(Integer::parseInt).toArray();
			elevatorMotorInvert = Boolean.parseBoolean(props.getProperty("elevatorMotorInvert"));
			elevatorPIDF = Stream.of(props.getProperty("elevatorPIDF").split(",")).mapToDouble(Double::parseDouble).toArray();			
			elevatorCruiseVelocity = Integer.parseInt(props.getProperty("elevatorCruiseVelocity"));
			elevatorAcceleration = Integer.parseInt(props.getProperty("elevatorAcceleration"));
			
			shifterPort = Integer.parseInt(props.getProperty("shifterPort"));
			ptoPort = Integer.parseInt(props.getProperty("ptoPort"));
			armPort = Integer.parseInt(props.getProperty("armPort"));
			intakePort = Integer.parseInt(props.getProperty("intakePort"));
			System.out.println("RobotMap: properties loaded!");
		} catch (Exception e) {
			System.out.println("RobotMap: Failed to load robot properties! " + e.getMessage());
			e.printStackTrace();
		}
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
