	package org.usfirst.frc.team1218.robot.commands.auton;

import org.usfirst.frc.team1218.robot.Robot;
import org.usfirst.frc.team1218.robot.Robot.Plate;
import org.usfirst.frc.team1218.robot.RobotMap;
import org.usfirst.frc.team1218.robot.commands.arm.ActuateArm;
import org.usfirst.frc.team1218.robot.commands.arm.ActuateIntakeArm;
import org.usfirst.frc.team1218.robot.commands.arm.ActuateIntakeWheels;
import org.usfirst.frc.team1218.robot.commands.arm.DropPowerCube;
import org.usfirst.frc.team1218.robot.commands.arm.ShootPowerCube;
import org.usfirst.frc.team1218.robot.commands.driveTrain.FollowPath;
import org.usfirst.frc.team1218.robot.commands.driveTrain.MotionMagicTurnToHeading;
import org.usfirst.frc.team1218.robot.commands.driveTrain.TalonFollowPath;
import org.usfirst.frc.team1218.robot.commands.driveTrain.WaitForProfilePointsRemaining;
import org.usfirst.frc.team1218.robot.commands.elevator.ElevatorMotionMagicMove;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class TwoCubeSwitchAutonSideCube extends CommandGroup {

    public TwoCubeSwitchAutonSideCube(Plate plate) {
    		addSequential(new SwitchAutonFast(plate));
    		if(plate == Robot.Plate.LEFT) {
    			RobotMap.leftSwitchMovingTurnPaths[0].goRight();
    			RobotMap.leftSwitchMovingTurnPaths[1].goLeft();
    			RobotMap.leftSwitchMovingTurnPaths[2].goRight();
    			RobotMap.leftSwitchMovingTurnPaths[3].goLeft();
    		}else {
    			RobotMap.leftSwitchMovingTurnPaths[3].goRight();
    			RobotMap.leftSwitchMovingTurnPaths[2].goLeft();
    			RobotMap.leftSwitchMovingTurnPaths[1].goRight();
    			RobotMap.leftSwitchMovingTurnPaths[0].goLeft();
    		}
    		addSequential(new TalonFollowPath(RobotMap.leftSwitchMovingTurnPaths[0],true));
    		addParallel(new ActuateIntakeWheels(1.0));
    		addParallel(new ActuateArm(false));
    		addSequential(new ActuateIntakeArm(true));
    		addParallel(new ElevatorMotionMagicMove(RobotMap.elevatorReverseLimit + 130));
    		addParallel(new TalonFollowPath(RobotMap.leftSwitchMovingTurnPaths[1],false));
    		addSequential(new WaitForProfilePointsRemaining(1));
    		addParallel(new ActuateIntakeArm(false));
    		addSequential(new TimedCommand(0.25));
    		addParallel(new ActuateArm(true));
    		addParallel(new ActuateIntakeWheels(0.5));
    		addParallel(new ElevatorMotionMagicMove(RobotMap.elevatorReverseLimit + 140));
    		addParallel(new TalonFollowPath(RobotMap.leftSwitchMovingTurnPaths[2],true));
    		addSequential(new WaitForProfilePointsRemaining(3));
    		addParallel(new ActuateIntakeWheels(0));
    		addParallel(new ElevatorMotionMagicMove(RobotMap.elevatorReverseLimit + 150));
    		addParallel(new TalonFollowPath(RobotMap.leftSwitchMovingTurnPaths[3],false));
    		addSequential(new WaitForProfilePointsRemaining(3));
    		addSequential(new DropPowerCube());
    		
        // Add Commands here:
        // e.g. addial(new Command1());
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
