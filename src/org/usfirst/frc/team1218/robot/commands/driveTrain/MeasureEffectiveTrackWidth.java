package org.usfirst.frc.team1218.robot.commands.driveTrain;

import org.usfirst.frc.team1218.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class MeasureEffectiveTrackWidth extends Command {

    public MeasureEffectiveTrackWidth() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.zeroPos();
    	Robot.driveTrain.startLogging();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double pwr = Robot.m_oi.driver.getY();
    	Robot.driveTrain.setPower(pwr, -pwr);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.stopLogging();
    	Robot.driveTrain.setPower(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() { 
    	end();
    }
    
}
