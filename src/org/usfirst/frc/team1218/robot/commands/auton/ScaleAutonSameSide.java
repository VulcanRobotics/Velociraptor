package org.usfirst.frc.team1218.robot.commands.auton;

import org.team1218.lib.trajectory.SimplePathGenerator;
import org.usfirst.frc.team1218.robot.Robot.Plate;
import org.usfirst.frc.team1218.robot.RobotMap;
import org.usfirst.frc.team1218.robot.commands.arm.ActuateArm;
import org.usfirst.frc.team1218.robot.commands.arm.ActuateIntakeArm;
import org.usfirst.frc.team1218.robot.commands.arm.ActuateIntakeWheels;
import org.usfirst.frc.team1218.robot.commands.arm.DropPowerCube;
import org.usfirst.frc.team1218.robot.commands.arm.ShootPowerCube;
import org.usfirst.frc.team1218.robot.commands.driveTrain.MotionMagicTurnToHeading;
import org.usfirst.frc.team1218.robot.commands.driveTrain.TalonFollowPath;
import org.usfirst.frc.team1218.robot.commands.driveTrain.WaitForProfilePointsRemaining;
import org.usfirst.frc.team1218.robot.commands.elevator.ElevatorMotionMagicMove;
import org.usfirst.frc.team1218.robot.commands.elevator.ElevatorMotionMagicMoveDelayed;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ScaleAutonSameSide extends CommandGroup {

    public ScaleAutonSameSide(Plate plate) {
    		TalonFollowPath pathCmd;
    		if(plate == Plate.RIGHT) {
    			pathCmd = new TalonFollowPath(RobotMap.rightStartRightScalePath);
    		}else {
    			pathCmd = new TalonFollowPath(RobotMap.leftStartLeftScalePath);
    		}
    		addParallel(new ElevatorMotionMagicMoveDelayed(750,2.5));
    		addParallel(pathCmd);
    		addSequential(new WaitForProfilePointsRemaining(10));
    		addSequential(new DropPowerCube());
    		addParallel(new ElevatorMotionMagicMove(RobotMap.elevatorReverseLimit + 3));
    		addSequential(new MotionMagicTurnToHeading(-180));
    		addParallel(new ActuateArm(false));
    		addParallel(new ActuateIntakeArm(true));
    		addParallel(new ActuateIntakeWheels(1.0));
    		addSequential(new TalonFollowPath(RobotMap.crossoverEnd,false));
    		addParallel(new ActuateArm(true));
    		addParallel(new ActuateIntakeArm(false));
    		addSequential(new MotionMagicTurnToHeading(0));
    		addParallel(new ElevatorMotionMagicMove(750));
    		addSequential(new TalonFollowPath(RobotMap.crossoverEnd,false));
    		addSequential(new DropPowerCube());
    		
        // e.g. (new Command1());
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
