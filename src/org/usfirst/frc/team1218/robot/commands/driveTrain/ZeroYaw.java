package org.usfirst.frc.team1218.robot.commands.driveTrain;

import org.usfirst.frc.team1218.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class ZeroYaw extends InstantCommand {

    public ZeroYaw() {
        super();
        requires(Robot.driveTrain);
    }

    // Called once when the command executes
    protected void initialize() {
    		Robot.driveTrain.navx.zeroYaw();
    }

}
