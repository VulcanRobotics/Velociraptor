package org.usfirst.frc.team1218.robot.commands.auton;

import org.usfirst.frc.team1218.robot.Robot;
import org.usfirst.frc.team1218.robot.RobotMap;
import org.usfirst.frc.team1218.robot.commands.driveTrain.FollowPath;
import org.usfirst.frc.team1218.robot.commands.elevator.DropPowerCube;
import org.usfirst.frc.team1218.robot.commands.elevator.ElevatorMotionMagicMove;
import org.usfirst.frc.team1218.robot.commands.elevator.ShootPowerCube;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ScaleAutonSide extends CommandGroup {

    public ScaleAutonSide() {
    		FollowPath pathCmd = Robot.followPathCmd;
    		pathCmd.setPath(RobotMap.leftStartleftScalePath, false);
    		addSequential(pathCmd);
    		addSequential(new ElevatorMotionMagicMove(245000));
    		addSequential(new ShootPowerCube());
    		addSequential(new ElevatorMotionMagicMove(0));
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