package org.usfirst.frc.team1218.robot.commands.elevator;

import org.usfirst.frc.team1218.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class actuateIntakeArm extends InstantCommand {
	
	boolean open;
	
	public actuateIntakeArm(boolean open) {
		this.open = open;
	}
	
	@Override
	protected void execute() {
		Robot.elevator.intakeSolenoidEngage(open);
	}
}
