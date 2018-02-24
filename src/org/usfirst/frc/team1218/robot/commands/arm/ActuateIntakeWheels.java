package org.usfirst.frc.team1218.robot.commands.arm;

import org.usfirst.frc.team1218.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class ActuateIntakeWheels extends InstantCommand {
	
	double power;
	
	public ActuateIntakeWheels(double power) {
		this.power = power;
	}
	
	@Override
	protected void execute() {
		Robot.arm.setIntakePower(power);
	}
}
