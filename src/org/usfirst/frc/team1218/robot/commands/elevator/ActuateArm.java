package org.usfirst.frc.team1218.robot.commands.elevator;

import org.usfirst.frc.team1218.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class ActuateArm extends InstantCommand {
	
	boolean up;
	
	public ActuateArm(boolean up) {
		this.up = up;
	}
	
	@Override
	protected void execute() {
		Robot.elevator.intakeSolenoidEngage(up);
	}
}
