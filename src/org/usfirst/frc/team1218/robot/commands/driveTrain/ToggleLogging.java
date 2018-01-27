package org.usfirst.frc.team1218.robot.commands.driveTrain;

import org.usfirst.frc.team1218.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class ToggleLogging extends InstantCommand {
	@Override
	public void execute() {
		if(Robot.driveTrain.isLogging()) {
			Robot.driveTrain.stopLogging();
		}else {
			Robot.driveTrain.startLogging();
		}
	}
}
