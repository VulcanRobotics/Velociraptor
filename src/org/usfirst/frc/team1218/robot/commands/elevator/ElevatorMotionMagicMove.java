package org.usfirst.frc.team1218.robot.commands.elevator;

import org.usfirst.frc.team1218.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ElevatorMotionMagicMove extends Command {

	private int position;
	
    public ElevatorMotionMagicMove(int position) {
    	requires(Robot.elevator);
    	this.position = position;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    		Robot.elevator.moveTo(position);
    		Robot.elevator.startLogging();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		System.out.println("Elevator Move to Pos Running, Error:" + Robot.elevator.getMotionMagicErr());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (Math.abs(Robot.elevator.getMotionMagicErr()) <= 2);
    }

    // Called once after isFinished returns true
    protected void end() {
    		Robot.elevator.stopLogging();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    		Robot.elevator.stopLogging();
    }
}
