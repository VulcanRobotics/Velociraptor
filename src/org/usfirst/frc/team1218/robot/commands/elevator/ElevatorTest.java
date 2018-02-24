package org.usfirst.frc.team1218.robot.commands.elevator;

import org.usfirst.frc.team1218.robot.commands.arm.ActuateArm;
import org.usfirst.frc.team1218.robot.commands.arm.ShootPowerCube;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class ElevatorTest extends CommandGroup {

    public ElevatorTest() {
    	 	addSequential(new ActuateArm(true));
        addSequential(new ElevatorMotionMagicMove(500));
        addSequential(new TimedCommand(0.25));
        addSequential(new ShootPowerCube());
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
