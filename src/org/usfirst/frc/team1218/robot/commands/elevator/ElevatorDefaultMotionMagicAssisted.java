package org.usfirst.frc.team1218.robot.commands.elevator;

import org.usfirst.frc.team1218.robot.Robot;
import org.usfirst.frc.team1218.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ElevatorDefaultMotionMagicAssisted extends Command {
	
	protected static final double deadband = 0.05;
	
	protected ControlMode controlMode = ControlMode.PercentOutput, lastControlMode = ControlMode.PercentOutput;
	protected int setpoint = RobotMap.elevatorReverseLimit;

    public ElevatorDefaultMotionMagicAssisted() {
    		requires(Robot.elevator);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		if(Math.abs(Robot.m_oi.operator.getY()) > deadband) {
    			controlMode = ControlMode.PercentOutput;
    		}else {
    			controlMode = ControlMode.MotionMagic;
    		}
    		
    		if(controlMode == ControlMode.MotionMagic && lastControlMode == ControlMode.PercentOutput) {
    			setpoint = Robot.elevator.getCurrentPosition();
    			if(setpoint < RobotMap.elevatorReverseLimit) {
    				setpoint = RobotMap.elevatorReverseLimit;
    			}else if(setpoint > RobotMap.elevatorForwardLimit - 10) {
    				setpoint = RobotMap.elevatorForwardLimit - 10;
    			}
    		}
    		
    		lastControlMode = controlMode;
    		
    		if(controlMode == ControlMode.PercentOutput) {
    			Robot.elevator.setElevatorPower(Robot.m_oi.operator.getY());
    		}else {
    			Robot.elevator.moveTo(setpoint);
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
