package org.usfirst.frc.team1218.robot.commands.driveTrain;

import org.usfirst.frc.team1218.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class WaitForProfilePointsRemaining extends Command {
	
	int numPoints;
	int counter = 0;
	
    public WaitForProfilePointsRemaining(int numPoints) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.numPoints = numPoints;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	counter = 0;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	counter ++;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	//System.out.println( Robot.driveTrain.getNumProfilePointLeft());
    	if(counter < 10) {
    		return false;
    	}else {
    		return Robot.driveTrain.getNumProfilePointLeft() < numPoints;
    	}
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
