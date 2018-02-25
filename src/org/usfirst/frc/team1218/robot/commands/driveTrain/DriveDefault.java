package org.usfirst.frc.team1218.robot.commands.driveTrain;

import org.usfirst.frc.team1218.robot.Robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveDefault extends Command {
	
	private static int count = 0;
	
    public DriveDefault() {
        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		double leftPower = -Robot.m_oi.driver.getY() + Math.pow(Robot.m_oi.driver.getX(),3);
    		double rightPower = -Robot.m_oi.driver.getY() - Math.pow(Robot.m_oi.driver.getX(),3);
    		Robot.driveTrain.setPower(leftPower, rightPower);
    		Robot.driveTrain.shift(Robot.m_oi.shiftBtn.get());
    		
    		// anti-lock braking, hopefully will minimize tipping
    		count += 1;
    		if (count >= 0 && count <= 2) Robot.driveTrain.setBrake(NeutralMode.Brake);
    		if (count >= 3 && count <= 5) Robot.driveTrain.setBrake(NeutralMode.Coast);
    		if (count >= 5) count = 0;
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
    		System.out.println("Interrupting DriveDefault....");
    }
}
