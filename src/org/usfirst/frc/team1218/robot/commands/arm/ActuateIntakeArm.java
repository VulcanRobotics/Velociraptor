package org.usfirst.frc.team1218.robot.commands.arm;

import org.usfirst.frc.team1218.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class ActuateIntakeArm extends InstantCommand {
	
	boolean open;
	
	public ActuateIntakeArm(boolean open) {
		this.open = open;
		requires(Robot.arm);
	}
	
	@Override
	protected void execute() {
		Robot.arm.intakeSolenoidEngage(open);
	}
}
