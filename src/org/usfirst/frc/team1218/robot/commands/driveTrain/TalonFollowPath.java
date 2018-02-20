package org.usfirst.frc.team1218.robot.commands.driveTrain;

import org.usfirst.frc.team1218.robot.Robot;

import com.team254.lib.trajectory.Path;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TalonFollowPath extends Command {
	
	Path path;
	int counter = 0;
	static final int delay = 1;
	
    public TalonFollowPath(Path path) {
        requires(Robot.driveTrain);
        this.path = path;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    		Robot.driveTrain.setPath(path, 1);
    		Robot.driveTrain.processMotionProfileBuffer();
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		if(counter == delay) {
    			Robot.driveTrain.startPath();
    		}
    		counter ++;
    		Robot.driveTrain.processMotionProfileBuffer();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (counter > delay) && (!Robot.driveTrain.isPathFollowing());
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
