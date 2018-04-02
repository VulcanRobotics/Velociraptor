package org.usfirst.frc.team1218.robot.commands.driveTrain;

import org.usfirst.frc.team1218.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class MotionMagicTurnToHeading extends Command {
	
	double angle = 0;
	int counter = 0;
	
    public MotionMagicTurnToHeading(double angle) {
        requires(Robot.driveTrain);
        this.angle = angle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.turnMotionMagic(Math.toRadians((Robot.driveTrain.getHeading()-angle)));
    	counter = 0;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(Robot.driveTrain.motionMagicOnTarget()) {
    		counter ++;
    	}else {
    		counter = 0;
    	}
        return counter >= 10;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
