package org.usfirst.frc.team1218.robot.commands.driveTrain;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

import org.usfirst.frc.team1218.robot.Robot;
import org.usfirst.frc.team1218.robot.subsystems.DriveTrain;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.Trajectory.Segment;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Test command to have the drive train follow a generated motion profile
 */
public class FollowPath extends Command {

	private Notifier processThread;
	private boolean useTalonMPMode, running, done, runBACKWARDS;
	private AtomicBoolean interrupt = new AtomicBoolean(false);
	private ArrayList<Segment> leftVelPts, rightVelPts;
	private double dtSeconds;
	
	private class PointExecutor implements Runnable {
		private long startTime;
		private boolean firstTime;
		private int step = 0;
		
		public PointExecutor() {
			firstTime = true;
		}

		private Segment invertSegment(Segment s) {
			return new Segment(-s.pos, -s.vel, -s.acc, -s.jerk, s.heading, s.dt, s.x, s.y);
		}
		
		public void run() {
	    	if (firstTime) {
	    		firstTime = false;
	    		startTime = System.currentTimeMillis();
	    		running = true;
	    		done = false;
	    	}
	    	step = (int)((System.currentTimeMillis() - startTime) / (long)(dtSeconds * 1000));
	    	//System.out.print("step: " + step);
	    	try {
	    		if (interrupt.get() == true) throw new Exception("Interrupting profile");
	    		if (runBACKWARDS){
	    			Robot.driveTrain.setVelocity(DriveTrain.ftPerSecToEncVel(rightVelPts.get(step).vel), 
	    										DriveTrain.ftPerSecToEncVel(leftVelPts.get(step).vel));	
	    		} else {
	    			Robot.driveTrain.setVelocity(DriveTrain.ftPerSecToEncVel(leftVelPts.get(step).vel), 
												DriveTrain.ftPerSecToEncVel(rightVelPts.get(step).vel));	
	    		}
	    	} catch (Exception e) {
	    		System.out.println("PointExecutor caught exception " + e.getMessage() + ", stopping Notifier");
	    		processThread.stop();
	    		running = false;
	    		done = true;
	    		firstTime = true;
	    	}
		}
	}
	
	public FollowPath() {
    	requires(Robot.driveTrain);
    	runBACKWARDS = false;
		leftVelPts = new ArrayList<Segment>();
		rightVelPts = new ArrayList<Segment>();
    	processThread = new Notifier(new PointExecutor());
    	running = false;
    	done = false;
    }
 
	public void setPath(Path p, boolean useTalonMPMode) {
    	if (running) {
    		System.out.println("FollowPath:  setPath() called while already running, ignoring");
    		return;
    	}
		this.useTalonMPMode = useTalonMPMode;		
    	leftVelPts.clear();
    	rightVelPts.clear();
    	//store the velocity pts
		int numPoints = p.getLeftWheelTrajectory().getNumSegments();
		Trajectory lt = p.getLeftWheelTrajectory(),
				   rt = p.getRightWheelTrajectory();
		for (int i = 0; i < numPoints; i++) {
			leftVelPts.add(lt.getSegment(i));
			rightVelPts.add(rt.getSegment(i));
			if (i==0) dtSeconds = lt.getSegment(i).dt;
		}

	}
	
    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("starting FollowPath command");
    	setPath(Robot.path,false);
    	processThread.startPeriodic(dtSeconds / 2.0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if (done) {
        	System.out.println("finished FollowPath command");
        	done = false;
        	return true;
        } else {
        	return false;
        }
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	if (running) {
    		interrupt.set(true);
    		System.out.println("Interrupting FollowPath");
    	}
    }
}

