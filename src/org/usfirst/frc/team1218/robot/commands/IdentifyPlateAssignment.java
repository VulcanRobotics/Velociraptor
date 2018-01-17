package org.usfirst.frc.team1218.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IdentifyPlateAssignment extends Command {
	
	
    public enum Plate {
    	LEFT,
    	RIGHT
	}
    static Plate ourSwitch; 
    static Plate scale; 
    static Plate theirSwitch; 


    public IdentifyPlateAssignment() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }


    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    		String gameData;
    		gameData = DriverStation.getInstance().getGameSpecificMessage();
    		//a string of 3 characters 'L'or'R', closest first from alliance station

    		if(gameData.charAt(0) == 'L') {
    			ourSwitch = Plate.LEFT;
    		} else {
    			ourSwitch = Plate.RIGHT;
    		}
    		
    		if(gameData.charAt(1) == 'L') {
    			scale = Plate.LEFT;
    		} else {
    			scale = Plate.RIGHT;
    		}
    		
    		if(gameData.charAt(2) == 'L') {
    			theirSwitch = Plate.LEFT;
    		} else {
    			theirSwitch = Plate.RIGHT;
    		}
    	
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
