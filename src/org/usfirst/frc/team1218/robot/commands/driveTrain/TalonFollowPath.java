package org.usfirst.frc.team1218.robot.commands.driveTrain;

import org.usfirst.frc.team1218.robot.Robot;
import org.usfirst.frc.team1218.robot.RobotMap;

import com.team254.lib.trajectory.Path;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TalonFollowPath extends Command {
	
	Path path;
	int counter = 0;
	static final int delay = 0;
	long startTime = 0;
	boolean gear;
	
	public TalonFollowPath(Path path) {
		this(path,false);
	}
	
    public TalonFollowPath(Path path, boolean gear) {
        requires(Robot.driveTrain);
        this.path = path;
        this.gear = gear;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    		Robot.driveTrain.shift(gear);
    		//TODO: uncomment after tuning is complete.
    		if(gear) {
    			//Robot.driveTrain.loadPIDFConstants(RobotMap.leftHighGearTalonMPPIDF, RobotMap.rightHighGearTalonMPPIDF);
    		}else {
    			//Robot.driveTrain.loadPIDFConstants(RobotMap.leftLowGearTalonMPPIDF, RobotMap.rightLowGearTalonMPPIDF);
    		}
    	
    		System.out.println("TalonPathFollower: Setting " + path.getName() + ".");
    		counter = 0;
    		Robot.driveTrain.setPath(path, 0.1);
    		Robot.driveTrain.processMotionProfileBuffer();
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//delay a little to make sure motion profiles is streamed in.
    	if(counter == delay) {
    		System.out.println("TalonPathFollower: Starting " + path.getName() + ".");
    		Robot.driveTrain.startPath();
    		startTime = System.currentTimeMillis();
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
    	System.out.println("TalonPathFollower: Completed " + path.getName() + "in " + (System.currentTimeMillis()-startTime) + "milliseconds.");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("TalonPathFollower: Interrupted " + path.getName() + ", ran for " + (System.currentTimeMillis()-startTime) + "milliseconds.");
    }
}
