package org.usfirst.frc.team1218.robot.commands.driveTrain;

import org.usfirst.frc.team1218.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class ToggleLogging extends InstantCommand{
	
	public ToggleLogging() {
		super();
		requires(Robot.driveTrain);
	}
	@Override
	public void execute() {
		boolean enableLogging = Robot.driveTrain.isLoggingEnabled();
		enableLogging = !enableLogging;
		Robot.driveTrain.setEnableLogging(enableLogging);
		System.out.println("Logging on Drive Train set to:" + enableLogging);
	}
}
