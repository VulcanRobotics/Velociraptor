/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1218.robot;

import org.team1218.lib.PropertiesManager;
import org.team1218.lib.trajectory.io.PathManager;

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
	
	public static double[] leftLowGearPIDF, leftLowGearTalonMPPIDF, leftHighGearPIDF, leftHighGearTalonMPPIDF,
						   leftMotionMagicPIDF,
						   rightLowGearPIDF, rightHighGearPIDF,rightLowGearTalonMPPIDF, rightHighGearTalonMPPIDF,
						   rightMotionMagicPIDF,
						   gyroTurnPIDF;
	public static double leftLowGearKv,		// = 0.7458140859603571, 
						leftLowGearKa,		// = 0.35191285889333246, //0.6971349598549942, 
						leftLowGearVInter, 	//= 0.6961006248023573, 
						rightLowGearKv,		// = 0.7554817379013181, 
						rightLowGearKa,		// = 0.3556168844830251, //0.7218365922908011, 
						rightLowGearVInter;	// = 0.8682917056968071;
	public static double leftHighGearKv, leftHighGearKa, leftHighGearVInter, rightHighGearKv, rightHighGearKa, rightHighGearVInter;
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
	public static double elevatorBottomInches, elevatorTopInches, elevatorTravalInches;
	
	public static TrajectoryGenerator.Config driveTrainPathConfig;
	
	public static Path centerStartRightSwitchPath, leftStartLeftScalePath, rightStartRightScalePath, centerStartLeftSwitchPath, tuningTestPath,
					   leftStartLeftSwitchPath, rightStartRightSwitchPath, leftStartStopEarlyPath, rightStartStopEarlyPath,rightStartLeftScalePath,
					   leftStartRightScalePath, crossoverStart, crossoverCross, crossoverEnd, centerStartRightSwitchReversePath, centerStartLeftSwitchReversePath,
					   twoCubeSwichPickupPath;
	
	public static void makePaths() {
		driveTrainPathConfig = new TrajectoryGenerator.Config();
		driveTrainPathConfig.dt = .1;			// the time in seconds between each generated segment
		driveTrainPathConfig.max_acc = 7.0;		// maximum acceleration for the trajectory, ft/s
		driveTrainPathConfig.max_jerk = 7.0;	// maximum jerk (derivative of acceleration), ft/s
		driveTrainPathConfig.max_vel = 6.0;		// maximum velocity you want the robot to reach for this trajectory, ft/s
				
		WaypointSequence ws = new WaypointSequence(10);
		/* right SWitch path with center start */
        ws.addWaypoint(new WaypointSequence.Waypoint(0.0, 0.0, 0.0));
        //ws.addWaypoint(new WaypointSequence.Waypoint(3.0, -2.0, 0.0));
        ws.addWaypoint(new WaypointSequence.Waypoint(7.0, -6, 0.0));
        centerStartRightSwitchPath = PathManager.getPath(ws, driveTrainPathConfig, trackWidthInches / 12.0, "rightSwitch");
        
        ws = new WaypointSequence(10);
        
		/* testing start left, left scale path */
		ws.addWaypoint(new WaypointSequence.Waypoint(0.0,0.0,0.0));
		ws.addWaypoint(new WaypointSequence.Waypoint(8, 0.0, 0.0));
		ws.addWaypoint(new WaypointSequence.Waypoint(17.5, -9, 0.0));
		driveTrainPathConfig.max_acc = 5.75;		// maximum acceleration for the trajectory, ft/s
		driveTrainPathConfig.max_jerk = 5.75;	// maximum jerk (derivative of acceleration), ft/s
		driveTrainPathConfig.max_vel = 5;		// maximum velocity you want the robot to reach for this trajectory, ft/s
		leftStartLeftScalePath = PathManager.getPath(ws, driveTrainPathConfig, trackWidthInches / 12.0, "leftStartLeftScale");
		
		ws = new WaypointSequence(10);
		
		ws.addWaypoint(new WaypointSequence.Waypoint(0.0,0.0,0.0));
		ws.addWaypoint(new WaypointSequence.Waypoint(15, 0.0, 0.0));
		ws.addWaypoint(new WaypointSequence.Waypoint(19, 4, Math.toRadians(60.0)));
		driveTrainPathConfig.max_acc = 5.75;		// maximum acceleration for the trajectory, ft/s
		driveTrainPathConfig.max_jerk = 5.75;	// maximum jerk (derivative of acceleration), ft/s
		driveTrainPathConfig.max_vel = 5.75;		// maximum velocity you want the robot to reach for this trajectory, ft/s
		rightStartRightScalePath = PathManager.getPath(ws, driveTrainPathConfig, trackWidthInches / 12.0, "rightStartRightScale");
		
		
		ws = new WaypointSequence(10);
		ws.addWaypoint(new WaypointSequence.Waypoint(0.0,0.0,0.0));
		ws.addWaypoint(new WaypointSequence.Waypoint(7.0,6,0.0));
		driveTrainPathConfig.max_acc = 7.0;		// maximum acceleration for the trajectory, ft/s
		driveTrainPathConfig.max_jerk = 7.0;	// maximum jerk (derivative of acceleration), ft/s
		driveTrainPathConfig.max_vel = 6.0;		// maximum velocity you want the robot to reach for this trajectory, ft/s
		centerStartLeftSwitchPath = PathManager.getPath(ws, driveTrainPathConfig, trackWidthInches / 12.0, "leftSwitch");
		
		/*tuning test path*/
		ws = new WaypointSequence(10);
		ws.addWaypoint(new WaypointSequence.Waypoint(0.0, 0.0, 0.0));
		ws.addWaypoint(new WaypointSequence.Waypoint(10.0, 2.0, Math.toRadians(45.0)));
		driveTrainPathConfig.max_acc = 7.0;		// maximum acceleration for the trajectory, ft/s
		driveTrainPathConfig.max_jerk = 7.0;	// maximum jerk (derivative of acceleration), ft/s
		driveTrainPathConfig.max_vel = 7.0;		// maximum velocity you want the robot to reach for this trajectory, ft/s
		tuningTestPath = PathManager.getPath(ws, driveTrainPathConfig, RobotMap.trackWidthInches / 12.0, "TuningTest");
		
		/* left Start left Switch */
		ws = new WaypointSequence(10);
		ws.addWaypoint(new WaypointSequence.Waypoint(0.0, 0.0, 0.0));
		ws.addWaypoint(new WaypointSequence.Waypoint(8.7, -3.3, Math.toRadians(70.0)));
		driveTrainPathConfig.max_acc = 7.0;		// maximum acceleration for the trajectory, ft/s
		driveTrainPathConfig.max_jerk = 7.0;	// maximum jerk (derivative of acceleration), ft/s
		driveTrainPathConfig.max_vel = 7.0;		// maximum velocity you want the robot to reach for this trajectory, ft/s
		leftStartLeftSwitchPath = PathManager.getPath(ws, driveTrainPathConfig, RobotMap.trackWidthInches / 12.0, "LeftStartLeftSwitch");

		/* right Start right Switch */
		ws = new WaypointSequence(10);
		ws.addWaypoint(new WaypointSequence.Waypoint(0.0, 0.0, 0.0));
		ws.addWaypoint(new WaypointSequence.Waypoint(9.0, 3.3, Math.toRadians(45.0)));
		driveTrainPathConfig.max_acc = 7.0;		// maximum acceleration for the trajectory, ft/s
		driveTrainPathConfig.max_jerk = 7.0;	// maximum jerk (derivative of acceleration), ft/s
		driveTrainPathConfig.max_vel = 7.0;		// maximum velocity you want the robot to reach for this trajectory, ft/s
		rightStartRightSwitchPath = PathManager.getPath(ws, driveTrainPathConfig, RobotMap.trackWidthInches / 12.0, "RightStartRightSwitch");

		/* Stop Early (for when we don't want to do switch if scale isn't ours */
		// currently both left and right just do the same straight path, we have both variables 
		// so each can be different if needed 
		ws = new WaypointSequence(10);
		ws.addWaypoint(new WaypointSequence.Waypoint(0.0, 0.0, 0.0));
		ws.addWaypoint(new WaypointSequence.Waypoint(14.0, 0.0, 0.0));
		driveTrainPathConfig.max_acc = 4.0;		// maximum acceleration for the trajectory, ft/s
		driveTrainPathConfig.max_jerk = 4.0;	// maximum jerk (derivative of acceleration), ft/s
		driveTrainPathConfig.max_vel = 4.0;		// maximum velocity you want the robot to reach for this trajectory, ft/s
		rightStartStopEarlyPath = PathManager.getPath(ws, driveTrainPathConfig, trackWidthInches / 12.0, "rightStartStopEarlyPath");
		
		ws = new WaypointSequence(10);
		ws.addWaypoint(new WaypointSequence.Waypoint(0.0, 0.0, 0.0));
		ws.addWaypoint(new WaypointSequence.Waypoint(14.0, 0.0, 0.0));
		driveTrainPathConfig.max_acc = 4.0;		// maximum acceleration for the trajectory, ft/s
		driveTrainPathConfig.max_jerk = 4.0;	// maximum jerk (derivative of acceleration), ft/s
		driveTrainPathConfig.max_vel = 4.0;		// maximum velocity you want the robot to reach for this trajectory, ft/s
		leftStartStopEarlyPath = PathManager.getPath(ws, driveTrainPathConfig, trackWidthInches / 12.0, "leftStartStopEarlyPath");
		
		ws = new WaypointSequence(3);
		ws.addWaypoint(new WaypointSequence.Waypoint(0.0,0.0,0.0));
		ws.addWaypoint(new WaypointSequence.Waypoint(12.0,0.0,0.0));
		ws.addWaypoint(new WaypointSequence.Waypoint(16,16.0,Math.toRadians(89.0)));
		driveTrainPathConfig.max_acc = 5.75;		// maximum acceleration for the trajectory, ft/s
		driveTrainPathConfig.max_jerk = 5.75;	// maximum jerk (derivative of acceleration), ft/s
		driveTrainPathConfig.max_vel = 5.75;		// maximum velocity you want the robot to reach for this trajectory, ft/s
		rightStartLeftScalePath = PathManager.getPath(ws, driveTrainPathConfig, trackWidthInches / 12.0, "rightStartLeftSwitchPath");
		
		ws = new WaypointSequence(3);
		ws.addWaypoint(new WaypointSequence.Waypoint(0.0,0.0,0.0));
		ws.addWaypoint(new WaypointSequence.Waypoint(12.0,0.0,0.0));
		ws.addWaypoint(new WaypointSequence.Waypoint(16,-16.0,Math.toRadians(-89.0)));
		driveTrainPathConfig.max_acc = 5.75;		// maximum acceleration for the trajectory, ft/s
		driveTrainPathConfig.max_jerk = 5.75;	// maximum jerk (derivative of acceleration), ft/s
		driveTrainPathConfig.max_vel = 5.75;		// maximum velocity you want the robot to reach for this trajectory, ft/s
		leftStartRightScalePath = PathManager.getPath(ws, driveTrainPathConfig, trackWidthInches / 12.0, "leftStartRightSwitchPath");
		
		ws = new WaypointSequence(2);
		ws.addWaypoint(new WaypointSequence.Waypoint(0.0,0.0,0.0));
		ws.addWaypoint(new WaypointSequence.Waypoint(18.0,0.0,0.0));
		//ws.addWaypoint(new WaypointSequence.Waypoint(5.0,0.0,0.0));
		driveTrainPathConfig.max_acc = 7.0;		// maximum acceleration for the trajectory, ft/s
		driveTrainPathConfig.max_jerk = 7.0;	// maximum jerk (derivative of acceleration), ft/s
		driveTrainPathConfig.max_vel = 7.0;		// maximum velocity you want the robot to reach for this trajectory, ft/s
		crossoverStart = PathManager.getPath(ws, driveTrainPathConfig, trackWidthInches / 12.0, "crossoverStart");
		
		ws = new WaypointSequence(2);
		ws.addWaypoint(new WaypointSequence.Waypoint(0.0,0.0,0.0));
		ws.addWaypoint(new WaypointSequence.Waypoint(17.0,0.0,0.0));
		//ws.addWaypoint(new WaypointSequence.Waypoint(3.0,0.0,0.0));
		driveTrainPathConfig.max_acc = 7.0;		// maximum acceleration for the trajectory, ft/s
		driveTrainPathConfig.max_jerk = 7.0;	// maximum jerk (derivative of acceleration), ft/s
		driveTrainPathConfig.max_vel = 7.0;		// maximum velocity you want the robot to reach for this trajectory, ft/s
		crossoverCross = PathManager.getPath(ws, driveTrainPathConfig, trackWidthInches / 12.0, "crossoverCross");
		
		ws = new WaypointSequence(2);
		ws.addWaypoint(new WaypointSequence.Waypoint(0.0,0.0,0.0));
		ws.addWaypoint(new WaypointSequence.Waypoint(4.5,0.0,0.0));
		driveTrainPathConfig.max_acc = 7.0;		// maximum acceleration for the trajectory, ft/s
		driveTrainPathConfig.max_jerk = 7.0;	// maximum jerk (derivative of acceleration), ft/s
		driveTrainPathConfig.max_vel = 7.0;		// maximum velocity you want the robot to reach for this trajectory, ft/s
		crossoverEnd = PathManager.getPath(ws, driveTrainPathConfig, trackWidthInches / 12.0, "crossoverEnd");
		
		ws = new WaypointSequence(2);
		ws.addWaypoint(new WaypointSequence.Waypoint(0.0,0.0,0.0));
		ws.addWaypoint(new WaypointSequence.Waypoint(7,-6,0.0));
		driveTrainPathConfig.max_acc = 7.0;		// maximum acceleration for the trajectory, ft/s
		driveTrainPathConfig.max_jerk = 7.0;	// maximum jerk (derivative of acceleration), ft/s
		driveTrainPathConfig.max_vel = 7.0;		// maximum velocity you want the robot to reach for this trajectory, ft/s
		centerStartLeftSwitchReversePath = PathManager.getPath(ws, driveTrainPathConfig, trackWidthInches / 12.0, "centerStartLeftSwitchReverse");
		
		ws = new WaypointSequence(2);
		ws.addWaypoint(new WaypointSequence.Waypoint(0.0,0.0,0.0));
		ws.addWaypoint(new WaypointSequence.Waypoint(7,6,0.0));
		driveTrainPathConfig.max_acc = 7.0;		// maximum acceleration for the trajectory, ft/s
		driveTrainPathConfig.max_jerk = 7.0;	// maximum jerk (derivative of acceleration), ft/s
		driveTrainPathConfig.max_vel = 6.0;		// maximum velocity you want the robot to reach for this trajectory, ft/s
		centerStartRightSwitchReversePath = PathManager.getPath(ws, driveTrainPathConfig, trackWidthInches / 12.0, "centerStartRightSwitchReverse");
		
		ws = new WaypointSequence(2);
		ws.addWaypoint(new WaypointSequence.Waypoint(0.0,0.0,0.0));
		ws.addWaypoint(new WaypointSequence.Waypoint(5.0,0.0,0.0));
		driveTrainPathConfig.max_acc = 7.0;		// maximum acceleration for the trajectory, ft/s
		driveTrainPathConfig.max_jerk = 7.0;	// maximum jerk (derivative of acceleration), ft/s
		driveTrainPathConfig.max_vel = 7.0;		// maximum velocity you want the robot to reach for this trajectory, ft/s
		twoCubeSwichPickupPath = PathManager.getPath(ws, driveTrainPathConfig, trackWidthInches / 12.0, "twoCubeSwitchPickup");
		
		driveTrainPathConfig.max_acc = 14.0;		// maximum acceleration for the trajectory, ft/s
		driveTrainPathConfig.max_jerk = 28.0;	// maximum jerk (derivative of acceleration), ft/s
		driveTrainPathConfig.max_vel = 7.0;
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
		leftLowGearTalonMPPIDF = pm.getDoubles("leftLowGearTalonMPPIDF",new double[] {0.0,0.0,0.0,0.79});
		rightLowGearTalonMPPIDF = pm.getDoubles("rightLowGearTalonMPPIDF",new double[] {0.0,0.0,0.0,0.79});
		leftHighGearTalonMPPIDF = pm.getDoubles("leftHighGearTalonMPPIDF", new double[] {0.0,0.0,0.0,0.0});
		rightHighGearTalonMPPIDF = pm.getDoubles("leftHighGearTalonMPPIDF", new double[] {0.0,0.0,0.0,0.0});
		rightLowGearPIDF = pm.getDoubles("rightLowGearPIDF", new double[] {1.05,0,50,0.79});
		leftHighGearPIDF = pm.getDoubles("leftHighGearPIDF", new double[] {0.0,0.0,0.0,0.0});
		rightHighGearPIDF = pm.getDoubles("rightHighGearPIDF", new double[] {0.0,0.0,0.0,0.0});
		leftMotionMagicPIDF = pm.getDoubles("leftMotionMagicPIDF", new double[] {2,0.0,20,0.3});
		rightMotionMagicPIDF = pm.getDoubles("rightMotionMagicPIDF", new double[] {2,0.0,20,0.35});
		gyroTurnPIDF = pm.getDoubles("gyroTurnPIDF", new double[] {0.0, 0.0, 0.0, 0.0});
		leftLowGearKv = pm.getDouble("leftLowGearKv");
		leftLowGearKa = pm.getDouble("leftLowGearKa");
		leftLowGearVInter = pm.getDouble("leftLowGearVInter");
		rightLowGearKv = pm.getDouble("rightLowGearKv");
		rightLowGearKa = pm.getDouble("rightLowGearKa");
		rightLowGearVInter = pm.getDouble("rightLowGearVInter");
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
		elevatorTopInches = pm.getDouble("elevatorTopInches", 52);
		elevatorBottomInches = pm.getDouble("elevatorBottomInches", 12);
		elevatorTravalInches = elevatorTopInches - elevatorBottomInches;
		
		shifterPort = pm.getInt("shifterPort");
		ptoPort = pm.getInt("ptoPort");
		armPort = pm.getInt("armPort");
		intakePort = pm.getInt("intakePort");
		System.out.println("RobotMap: properties loaded!");
		System.out.println("lt Kv: " + leftLowGearKv + " lt Ka: " + leftLowGearKa + " lt VInter: " + leftLowGearVInter);
		System.out.println("rt Kv: " + rightLowGearKv + " rt Ka: " + rightLowGearKa + " rt VInter: " + rightLowGearVInter);
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
