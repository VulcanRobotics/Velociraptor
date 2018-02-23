package org.usfirst.frc.team1218.robot.commands.elevator;

import org.usfirst.frc.team1218.robot.Robot;
import org.usfirst.frc.team1218.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ElevatorDefaultMotionMagic extends Command {

    public ElevatorDefaultMotionMagic() {
    	requires(Robot.elevator);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double position = Robot.elevator.getCurrentPosition() + (Robot.m_oi.operator.getY() * (RobotMap.elevatorCruiseVelocity / 5.0));
    	if (position < 0) position = 0;
    	if (position > 290000) position = 290000;
    	
    	Robot.elevator.moveTo((int)position);
    	
		if(Robot.m_oi.intakeBtn.get()) {
			Robot.elevator.setIntakePower(1);
		}else if(Robot.m_oi.outtakeBtn.get()){
			Robot.elevator.setIntakePower(-0.70);
		}else {
			Robot.elevator.setIntakePower(0);
		}
		
		
		Robot.elevator.intakeSolenoidEngage(Robot.m_oi.intakeArmBtn.get());
		
		if(Robot.m_oi.armUpBtn.get()) {
			Robot.elevator.armSolenoidEngage(true);
		}else if(Robot.m_oi.armDownBtn.get()) {
			Robot.elevator.armSolenoidEngage(false);
		}

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
