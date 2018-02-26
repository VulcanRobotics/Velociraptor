/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1218.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import org.eclipse.jetty.server.Handler;
import org.eclipse.jetty.server.Server;
import org.eclipse.jetty.server.ServerConnector;
import org.eclipse.jetty.server.handler.DefaultHandler;
import org.eclipse.jetty.server.handler.HandlerList;
import org.eclipse.jetty.server.handler.ResourceHandler;
import org.usfirst.frc.team1218.robot.commands.auton.ScaleAutonSide;
import org.usfirst.frc.team1218.robot.commands.auton.SwitchAuton;
import org.usfirst.frc.team1218.robot.commands.driveTrain.FollowPath;
import org.usfirst.frc.team1218.robot.commands.driveTrain.TalonFollowPath;
import org.usfirst.frc.team1218.robot.subsystems.Arm;
import org.usfirst.frc.team1218.robot.subsystems.DriveTrain;
import org.usfirst.frc.team1218.robot.subsystems.Elevator;
import org.usfirst.frc.team1218.robot.subsystems.ElevatorEnc;
import org.usfirst.frc.team1218.robot.subsystems.ElevatorPot;

import com.team254.lib.trajectory.Path;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	
	public enum RobotStartingPosition{
		left,center,right;
	}
	
	public static final int outSwitch = 0, scale = 1, theirSwitch = 2;
	
	public enum Plate {
    		LEFT,
    		RIGHT
	}
	
	public static OI m_oi;
	public static DriveTrain driveTrain;
	public static Elevator elevator;
	public static Arm arm;
	public static Plate plateAssignments[] = new Plate[3];
	
	
	private static UsbCamera jevois;
	public static FollowPath followPathCmd;
	public static RobotStartingPosition robotStartingPos = RobotStartingPosition.center;
	
	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();
	public static Path path;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		RobotMap.loadProperties();
		RobotMap.makePaths();
		driveTrain = new DriveTrain();
		if(RobotMap.useEncElevator) {
			elevator = new ElevatorEnc();
		}else {
			elevator = new ElevatorPot();
		}
		arm = new Arm();
		//followPathCmd = new FollowPath();
		//followPathCmd.setPath(RobotMap.tuningTestPath, false);
		m_oi = new OI();
        m_oi.followPathBtn.whenPressed(/*followPathCmd*/new TalonFollowPath(RobotMap.tuningTestPath));

        if (RobotMap.useCamera) {
        	jevois = CameraServer.getInstance().startAutomaticCapture();
        	jevois.setVideoMode(PixelFormat.kMJPEG,320,240,30);
        	VideoMode vm = jevois.getVideoMode();
        	System.out.println("jevois pixel: " + vm.pixelFormat);
        	System.out.println("jevois res: " + vm.width + "x" + vm.height);
        	System.out.println("jevois fps: " + vm.fps);
        } 
        
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", m_chooser);
		Server jettyServer = new Server(5800);
        ServerConnector connector = new ServerConnector(jettyServer);
        connector.setPort(5800);
        jettyServer.addConnector(connector);

        ResourceHandler resource_handler = new ResourceHandler();
        resource_handler.setDirectoriesListed(true);
        resource_handler.setWelcomeFiles(new String[]{ "webroot/index.html" });
        
        resource_handler.setResourceBase("/home/lvuser");
        
        HandlerList handlers = new HandlerList();
        handlers.setHandlers(new Handler[] { resource_handler, new DefaultHandler() });
        jettyServer.setHandler(handlers);
        try {
			jettyServer.start();
			jettyServer.join();
		} catch (Exception e1) {
			// TODO Auto-generated catch block
//			e1.printStackTrace();
		}

		
	}
	

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		periodicTasks();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_chooser.getSelected();
		identifyPlateAssignment();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		periodicTasks();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		periodicTasks();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	
	public void periodicTasks() {
		driveTrain.periodicTasks();
		elevator.periodicTasks();
		
	}
	
	public void identifyPlateAssignment() {
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		System.out.println("Recived Game Message: " + gameData);
		System.out.print("Assignment Identified as: ");
		for(int i = 0; i < 3; i++) {
			if(gameData.charAt(i) == 'L') {
				plateAssignments[i] = Plate.LEFT;
				System.out.print(" [ LEFT ] ");
			}else{
				plateAssignments[i] = Plate.RIGHT;
				System.out.print(" [ RIGHT ] ");
			}
		}
		System.out.println();
	}
}
