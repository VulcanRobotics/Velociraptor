package org.usfirst.frc.team1218.robot.commands.driveTrain;

import org.usfirst.frc.team1218.robot.Robot;
import org.usfirst.frc.team1218.robot.RobotMap;
import org.usfirst.frc.team1218.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnToAngle extends Command {

	double encoderTicksToTurn;
	double radiansToInches;
	
    public TurnToAngle(double angleInRadians) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
    	requires(Robot.driveTrain);
    	 
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute(double angleInRadians) {
    	
    double inchesToTurn = DriveTrain.radiansToInches(angleInRadians);
    double encoderTicksToTurn = ((DriveTrain.wheelDiameterInches*(Math.PI))/RobotMap.encTicksPerRev);
    if(angleInRadians > 0) {
	    	double leftPower = 0.5;
	    	double rightPower = -0.5;
	    	Robot.driveTrain.setPower(leftPower, rightPower);
	    	Robot.driveTrain.shift(Robot.m_oi.shiftBtn.get());
	    	//Drive for encoderTicksToTurn
    }else {
	    	double leftPower = -0.5;
	    	double rightPower = 0.5;
	    	Robot.driveTrain.setPower(leftPower, rightPower);
	    	Robot.driveTrain.shift(Robot.m_oi.shiftBtn.get());
	    	//Drive for encoderTicksToTurn
    }
	
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
