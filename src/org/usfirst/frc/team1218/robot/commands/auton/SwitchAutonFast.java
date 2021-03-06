package org.usfirst.frc.team1218.robot.commands.auton;

import org.usfirst.frc.team1218.robot.Robot;
import org.usfirst.frc.team1218.robot.Robot.Plate;
import org.usfirst.frc.team1218.robot.RobotMap;
import org.usfirst.frc.team1218.robot.commands.arm.DropPowerCube;
import org.usfirst.frc.team1218.robot.commands.arm.ShootPowerCube;
import org.usfirst.frc.team1218.robot.commands.driveTrain.FollowPath;
import org.usfirst.frc.team1218.robot.commands.driveTrain.TalonFollowPath;
import org.usfirst.frc.team1218.robot.commands.driveTrain.WaitForProfilePointsRemaining;
import org.usfirst.frc.team1218.robot.commands.elevator.ElevatorMotionMagicMove;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class SwitchAutonFast extends CommandGroup {

    public SwitchAutonFast(Plate plate) {
    		TalonFollowPath pathCmd;
    		if(plate == Plate.RIGHT) {
    			pathCmd = new TalonFollowPath(RobotMap.centerStartRightSwitchPath);
    		}else {
    			pathCmd = new TalonFollowPath(RobotMap.centerStartLeftSwitchPath);
    		}
    		addParallel(pathCmd);
    		addSequential(new ElevatorMotionMagicMove(RobotMap.elevatorReverseLimit + 150));
    		addSequential(new WaitForProfilePointsRemaining(7));
    		addSequential(new DropPowerCube());
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
