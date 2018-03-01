package org.usfirst.frc.team1218.robot.commands.elevator;

import org.usfirst.frc.team1218.robot.Robot;
import org.usfirst.frc.team1218.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ElevatorMotionMagicMoveInches extends Command {

	private double position;
	
    public ElevatorMotionMagicMoveInches(double position) {
    	requires(Robot.elevator);
    	this.position = position;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    		Robot.elevator.moveToInches(position);
    		Robot.elevator.startLogging();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    		//allow 5% error
        return (Math.abs(Robot.elevator.getMotionMagicErr()) <= RobotMap.elevatorTraval*0.05);
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
