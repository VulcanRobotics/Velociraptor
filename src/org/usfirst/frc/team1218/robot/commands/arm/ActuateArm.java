package org.usfirst.frc.team1218.robot.commands.arm;

import org.usfirst.frc.team1218.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class ActuateArm extends InstantCommand {
	
	boolean up;
	
	public ActuateArm(boolean up) {
		this.up = up;
		requires(Robot.arm);
	}
	
	@Override
	protected void execute() {
		Robot.arm.armSolenoidEngage(up);
	}
}
