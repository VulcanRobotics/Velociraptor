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
    	double js = Robot.m_oi.operator.getY();
    	double position = Robot.elevator.getTargetPosition();
    	if (Math.abs(js) >= .05) {
    		position += js * (35.0 / 2.0);
    	}
    	if (position < 185) position = 185;
    	if (position > 900) position = 900;
    	
    	Robot.elevator.setMotionMagicSpeeds((int)Math.abs(js) * 35);
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
