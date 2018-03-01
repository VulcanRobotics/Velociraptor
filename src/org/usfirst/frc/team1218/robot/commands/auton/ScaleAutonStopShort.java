package org.usfirst.frc.team1218.robot.commands.auton;

import org.usfirst.frc.team1218.robot.RobotMap;
import org.usfirst.frc.team1218.robot.commands.driveTrain.TalonFollowPath;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ScaleAutonStopShort extends CommandGroup {
	
	public ScaleAutonStopShort() {
		addSequential(new TalonFollowPath(RobotMap.leftStartStopEarlyPath));
	}
}
