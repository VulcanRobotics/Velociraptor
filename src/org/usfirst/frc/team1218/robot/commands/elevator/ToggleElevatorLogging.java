package org.usfirst.frc.team1218.robot.commands.elevator;

import org.usfirst.frc.team1218.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class ToggleElevatorLogging extends InstantCommand {
	@Override
	public void execute() {
		if(Robot.elevator.isLogging()) {
			Robot.elevator.stopLogging();
		}else {
			Robot.elevator.startLogging();
		}
	}
}
