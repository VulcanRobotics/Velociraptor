package org.usfirst.frc.team1218.robot.commands.auton;

import org.team1218.lib.trajectory.SimplePathGenerator;
import org.usfirst.frc.team1218.robot.Robot.Plate;
import org.usfirst.frc.team1218.robot.RobotMap;
import org.usfirst.frc.team1218.robot.commands.arm.DropPowerCube;
import org.usfirst.frc.team1218.robot.commands.arm.ShootPowerCube;
import org.usfirst.frc.team1218.robot.commands.driveTrain.MotionMagicTurn;
import org.usfirst.frc.team1218.robot.commands.driveTrain.TalonFollowPath;
import org.usfirst.frc.team1218.robot.commands.elevator.ElevatorMotionMagicMove;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ScaleAutonCrossOver extends CommandGroup {

    public ScaleAutonCrossOver(Plate plate) {
    		addSequential(new TalonFollowPath(RobotMap.crossoverStart, false));
    		//addSequential(new TalonFollowPath(SimplePathGenerator.generateTurn(Math.PI/2.0*1.25, RobotMap.driveTrainPathConfig, RobotMap.trackWidthInches/12.0)));
    		addSequential(new MotionMagicTurn(-Math.PI/2.0));
    		addSequential(new TalonFollowPath(RobotMap.crossoverCross, false));
    		addSequential(new MotionMagicTurn(Math.PI/2.0));
    		//addSequential(new TalonFollowPath(SimplePathGenerator.generateTurn(-Math.PI/2.0*1.25, RobotMap.driveTrainPathConfig, RobotMap.trackWidthInches/12.0)));
    		addParallel(new TalonFollowPath(RobotMap.crossoverEnd, false));
    		addSequential(new ElevatorMotionMagicMove(700));
    		addSequential(new DropPowerCube());
    		//addSequential(new ElevatorMotionMagicMove(0));
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
