package org.usfirst.frc.team1218.robot.commands.auton;

import org.usfirst.frc.team1218.robot.Robot;
import org.usfirst.frc.team1218.robot.Robot.Plate;
import org.usfirst.frc.team1218.robot.Robot.RobotStartingPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonLauncher extends CommandGroup {

    public AutonLauncher(RobotStartingPosition pos) {
    		if(pos == RobotStartingPosition.center) {
    			addSequential(new SwitchAuton(Robot.plateAssignments[Robot.outSwitch]));
    		}else if(pos == RobotStartingPosition.left){
    			if(Robot.plateAssignments[Robot.scale] == Plate.LEFT) {
    				addSequential(new ScaleAutonSameSide(Robot.plateAssignments[Robot.scale]));
    			}else if(Robot.plateAssignments[Robot.outSwitch] == Plate.LEFT){
    				addSequential(new SwitchSideAuton(Robot.plateAssignments[Robot.outSwitch]));
    			}else {
    				addSequential(new ScaleAutonCrossOver(null));
    			}
    		}else {
    			if(Robot.plateAssignments[Robot.scale] == Plate.RIGHT) {
    				addSequential(new ScaleAutonSameSide(Robot.plateAssignments[Robot.scale]));
    			}else if(Robot.plateAssignments[Robot.outSwitch] == Plate.RIGHT){
    				addSequential(new SwitchSideAuton(Robot.plateAssignments[Robot.outSwitch]));
    			}else {
    				addSequential(new ScaleAutonStopShort());
    			}
    		}
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }
}
