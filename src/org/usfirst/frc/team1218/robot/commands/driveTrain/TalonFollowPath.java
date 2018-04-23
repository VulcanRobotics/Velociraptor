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
	boolean reverse;
	
	public TalonFollowPath(Path path) {
		this(path,false,false);
	}
	
	public TalonFollowPath(Path path,boolean reverse) {
		this(path, false, reverse);
	}
	
    public TalonFollowPath(Path path, boolean gear, boolean reverse) {
        requires(Robot.driveTrain);
        this.path = path;
        this.gear = gear;
        this.reverse = reverse;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    		//TODO: uncomment after tuning is complete.
    		Robot.driveTrain.loadPIDFConstants(RobotMap.leftLowGearTalonMPPIDF, RobotMap.rightLowGearTalonMPPIDF);
    	
    		System.out.println("TalonPathFollower: Setting " + path.getName() + ".");
    		counter = 0;
    		Robot.driveTrain.setPath(path, 0.1,reverse);
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
    	
    	if (counter == path.getLeftWheelTrajectory().getNumSegments() && gear == true) {
    		Robot.driveTrain.loadPIDFConstants(RobotMap.leftHighGearTalonMPPIDF, RobotMap.rightHighGearTalonMPPIDF);
    		Robot.driveTrain.shift(true);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (counter > delay) && (!Robot.driveTrain.isPathFollowing());
    }

    // Called once after isFinished returns true
    protected void end() {
    		System.out.println("TalonPathFollower: Completed " + path.getName() + "in " + (System.currentTimeMillis()-startTime) + "milliseconds.");
    		stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    		System.out.println("TalonPathFollower: Interrupted " + path.getName() + ", ran for " + (System.currentTimeMillis()-startTime) + "milliseconds.");
    		stop();
    }
    
    protected void stop() {
    		Robot.driveTrain.setPower(0, 0);
    }
}
