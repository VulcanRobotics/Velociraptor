package org.usfirst.frc.team1218.robot.commands.arm;

import org.usfirst.frc.team1218.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmDefault extends Command {

    public ArmDefault() {
        requires(Robot.arm);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		if(!DriverStation.getInstance().isAutonomous()) {
    			if(Robot.m_oi.intakeBtn.get()) {
    				Robot.arm.setIntakePower(1);
    			}else if(Robot.m_oi.outtakeBtn.get()){
    				Robot.arm.setIntakePower(-0.70);
    			}else {
    				Robot.arm.setIntakePower(0);
    			}
		
		
    			Robot.arm.intakeSolenoidEngage(Robot.m_oi.intakeArmBtn.get());
		
    			if(Robot.m_oi.armUpBtn.get()) {
    				Robot.arm.armSolenoidEngage(true);
    			}else if(Robot.m_oi.armDownBtn.get()) {
    				Robot.arm.armSolenoidEngage(false);
    			}
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
