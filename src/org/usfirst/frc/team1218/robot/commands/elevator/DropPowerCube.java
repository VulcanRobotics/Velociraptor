package org.usfirst.frc.team1218.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class DropPowerCube extends CommandGroup {

    public DropPowerCube() {
    		addSequential(new ActuateIntakeArm(true));
    		addSequential(new TimedCommand(0.25));
    		addSequential(new ActuateIntakeWheels(-0.75));
    		addSequential(new TimedCommand(1));
    		addSequential(new ActuateIntakeWheels(0));
    		addSequential(new ActuateIntakeArm(false));
       
    }
}
