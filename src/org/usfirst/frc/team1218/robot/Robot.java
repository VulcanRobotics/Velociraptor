/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1218.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.net.URI;
import java.net.URL;

import org.eclipse.jetty.server.Server;
import org.eclipse.jetty.servlet.ServletContextHandler;
import org.eclipse.jetty.util.resource.Resource;
import org.usfirst.frc.team1218.robot.commands.driveTrain.FollowPath;
import org.usfirst.frc.team1218.robot.subsystems.DriveTrain;

import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.PathGenerator;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.WaypointSequence;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static OI m_oi;
	public static DriveTrain driveTrain;

	Command m_autonomousCommand;
	FollowPath followPathCmd;
	SendableChooser<Command> m_chooser = new SendableChooser<>();
	public static Path path;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		driveTrain = new DriveTrain(RobotMap.leftMotorControllerIds,RobotMap.rightMotorControllerIds,RobotMap.leftInverted,RobotMap.rightInverted,RobotMap.shifterPort);
		m_oi = new OI();
		followPathCmd = new FollowPath();
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", m_chooser);
		Server jettyServer = new Server(5800);
		URL webRootLocation = this.getClass().getResource("/webroot/index.html");
		if (webRootLocation == null) {
			System.out.println("Got NULL trying to load webroot resource from jar");
		} else {
			try {
				URI webRootUri = URI.create(webRootLocation.toURI().toASCIIString().replaceFirst("/index.html$","/"));
				System.out.println("Web Root URI: " + webRootUri);

				ServletContextHandler context = new ServletContextHandler();
				context.setContextPath("/");
				context.setBaseResource(Resource.newResource(webRootUri));
				context.setWelcomeFiles(new String[] { "index.html" });
				context.getMimeTypes().addMimeMapping("txt","text/plain;charset=utf-8");
				jettyServer.setHandler(context);
	        
				jettyServer.start();
				jettyServer.join();
				System.out.println("Started web server on port 5800");
			} catch (Exception e) {
				System.out.println("Exception " + e.getMessage() + " starting web server on port 5800");
				e.printStackTrace();
			}
		}
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		
		TrajectoryGenerator.Config config = new TrajectoryGenerator.Config();
		config.dt = .1;			// the time in seconds between each generated segment
		config.max_acc = 7.0;		// maximum acceleration for the trajectory, ft/s
		config.max_jerk = 30.0;	// maximum jerk (derivative of acceleration), ft/s
		config.max_vel = 7.0;		// maximum velocity you want the robot to reach for this trajectory, ft/s

		WaypointSequence ws = new WaypointSequence(10);
        ws.addWaypoint(new WaypointSequence.Waypoint(0.0, 0.0, 0.0));
        ws.addWaypoint(new WaypointSequence.Waypoint(5.0, 0.0, 0.0));
        path = PathGenerator.makePath(ws, config,
                driveTrain.trackWidthInches / 12.0, "Test Drive 5ft");
        /*followPathCmd.setPath(PathGenerator.makePath(ws, config,
                driveTrain.trackWidthInches / 12.0, "Test Drive 5ft"),false);*/
        m_oi.followPathBtn.whenPressed(new FollowPath());
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
		
	}
}
