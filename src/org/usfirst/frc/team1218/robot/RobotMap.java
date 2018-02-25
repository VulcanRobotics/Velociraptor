/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1218.robot;

import org.team1218.lib.PropertiesManager;

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
	public static boolean useEncElevator;
	
	public static double[] leftLowGearPIDF, leftLowGearTalonMPPIDF, leftHighGearPIDF, rightLowGearPIDF, rightHighGearPIDF, rightLowGearTalonMPPIDF;
	public static int lowGearMaxSpeed, highGearMaxSpeed, encTicksPerRev;
	public static double trackWidthInches;
	
	public static int shifterPort, ptoPort, intakePort, armPort;
	
	public static int[] intakeMotorIds;				// {4,11};
	public static boolean[] intakeMotorInvert; 		// {true,false};
	public static int[] elevatorMotorIds;			//{3,12};
	public static boolean elevatorMotorInvert; 		// true;
	public static double[] elevatorPIDF;
	public static int elevatorCruiseVelocity, elevatorAcceleration;
	public static int elevatorReverseLimit, elevatorForwardLimit, elevatorTraval;
	
	public static Path rightSwitchPath, leftStartleftScalePath, tuningTestPath;
	
	public static void makePaths() {
		TrajectoryGenerator.Config driveTrainPathConfig = new TrajectoryGenerator.Config();
		driveTrainPathConfig.dt = .1;			// the time in seconds between each generated segment
		driveTrainPathConfig.max_acc = 14.0;		// maximum acceleration for the trajectory, ft/s
		driveTrainPathConfig.max_jerk = 28.0;	// maximum jerk (derivative of acceleration), ft/s
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
		
		ws = new WaypointSequence(10);
		
		/*tuning test stright line*/
		ws.addWaypoint(new WaypointSequence.Waypoint(0.0, 0.0, 0.0));
		ws.addWaypoint(new WaypointSequence.Waypoint(5.0, 0.0, 0.0));
		tuningTestPath = PathGenerator.makePath(ws, driveTrainPathConfig, RobotMap.trackWidthInches,"Tuning Test Path");
	}
	
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
			
		useCamera = pm.getBoolean("useCamera",true);
		useEncElevator = pm.getBoolean("useEncElevator",false);
			
		elevatorMotorIds = pm.getInts("elevatorMotorIds", new int[] {3,12});
		elevatorMotorInvert = pm.getBoolean("elevatorMotorInvert",false);
		
		if(useEncElevator) {
			elevatorPIDF = pm.getDoubles("elevatorEncPIDF", new double[] {0.0,0.0,0.0,0.0});			
			elevatorCruiseVelocity = pm.getInt("elevatorEncCruiseVelocity");
			elevatorAcceleration = pm.getInt("elevatorEncAcceleration");
			elevatorForwardLimit = pm.getInt("elevatorEncForwardLimit",295000);
			elevatorReverseLimit = 0;
		}else {
			elevatorPIDF = pm.getDoubles("elevatorPotPIDF", new double[] {7.5,0.0,55.0,29.228});
			elevatorCruiseVelocity = pm.getInt("elevatorPotCruiseVelocity",35);
			elevatorAcceleration = pm.getInt("elevatorPotAcceleration",70);
			elevatorForwardLimit = pm.getInt("elevatorPotForwardLimit",900);
			elevatorReverseLimit = pm.getInt("elevatorPotReverseLimit",185);
		}
		elevatorTraval = elevatorForwardLimit - elevatorReverseLimit;
		
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
