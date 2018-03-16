package org.usfirst.frc.team1218.robot.commands.auton;

import org.team1218.lib.trajectory.SimplePathGenerator;
import org.usfirst.frc.team1218.robot.Robot.Plate;
import org.usfirst.frc.team1218.robot.RobotMap;
import org.usfirst.frc.team1218.robot.commands.arm.ShootPowerCube;
import org.usfirst.frc.team1218.robot.commands.driveTrain.TalonFollowPath;
import org.usfirst.frc.team1218.robot.commands.elevator.ElevatorMotionMagicMove;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ScaleAutonCrossOver extends CommandGroup {

    public ScaleAutonCrossOver(Plate plate) {
    		TalonFollowPath pathCmd;
    		TalonFollowPath turnCmd;
    		if(plate == Plate.RIGHT) {
    			pathCmd = new TalonFollowPath(RobotMap.leftStartRightScalePath);
    			turnCmd = new TalonFollowPath(SimplePathGenerator.generateTurn(Math.toRadians(-91.0), RobotMap.driveTrainPathConfig, RobotMap.trackWidthInches/12.0));
    		}else {
    			pathCmd = new TalonFollowPath(RobotMap.rightStartLeftScalePath);
    			turnCmd = new TalonFollowPath(SimplePathGenerator.generateTurn(Math.toRadians(91.0), RobotMap.driveTrainPathConfig, RobotMap.trackWidthInches/12.0));
    		}
    		addSequential(pathCmd);
    		addSequential(turnCmd);
    		addParallel(new TalonFollowPath(SimplePathGenerator.generateLine(3.0, RobotMap.driveTrainPathConfig)));
    		addSequential(new ElevatorMotionMagicMove(700));
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
