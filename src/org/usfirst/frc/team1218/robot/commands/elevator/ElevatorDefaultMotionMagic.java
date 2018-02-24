package org.usfirst.frc.team1218.robot.commands.elevator;

import org.usfirst.frc.team1218.robot.Robot;
import org.usfirst.frc.team1218.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ElevatorDefaultMotionMagic extends Command {

    public ElevatorDefaultMotionMagic() {
    	requires(Robot.elevator);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double position = Robot.elevator.getCurrentPosition() + (Robot.m_oi.operator.getY() * (RobotMap.elevatorCruiseVelocity / 5.0));
    	if (position < 0) position = 0;
    	if (position > 290000) position = 290000;
    	
    	Robot.elevator.moveTo((int)position);

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
