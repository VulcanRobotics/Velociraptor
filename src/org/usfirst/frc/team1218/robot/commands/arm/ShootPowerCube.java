package org.usfirst.frc.team1218.robot.commands.arm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class ShootPowerCube extends CommandGroup {

    public ShootPowerCube() {
    	addSequential(new ActuateIntakeArm(false));
    		addSequential(new ActuateIntakeWheels(-0.4));
    		addSequential(new TimedCommand(1));
    		addSequential(new ActuateIntakeWheels(0));
    }
}
