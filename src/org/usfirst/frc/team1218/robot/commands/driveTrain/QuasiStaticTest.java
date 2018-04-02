package org.usfirst.frc.team1218.robot.commands.driveTrain;

import java.io.BufferedWriter;

import org.usfirst.frc.team1218.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import mjson.Json;

/**
 * collects drive train characterization data as per Oblarg's paper
 */
public class QuasiStaticTest extends Command {
	
	private int savedRampRate = 0;
    public QuasiStaticTest() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Starting CharacterizeDriveTrain()");
    	Robot.driveTrain.configOpenLoopRampRate(240);
    	Robot.driveTrain.startLogging();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveTrain.setPower(1.0, 1.0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("Ending CharacterizeDriveTrain()");
    	Robot.driveTrain.stopLogging();
    	Robot.driveTrain.configOpenLoopRampRate(0);
    	Robot.driveTrain.setPower(0.0, 0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
