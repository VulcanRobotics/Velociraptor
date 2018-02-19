package org.usfirst.frc.team1218.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DescriptiveCommandName extends Command {

    public DescriptiveCommandName() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    		System.out.println("Descriptive Command Initialized");
    		System.out.println("This is a very");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		System.out.println("very");
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    		System.out.println("desscriptive Command.");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    		end();
    }
}
