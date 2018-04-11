package org.usfirst.frc.team1218.robot.commands.elevator;

import org.usfirst.frc.team1218.robot.Robot;
import org.usfirst.frc.team1218.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ElevatorDefaultMotionMagicAssistedRamped extends Command {
	
	protected static final double deadband = 0.2;
	protected static final double rampStep = 0.02;
	
	protected ControlMode controlMode = ControlMode.PercentOutput, lastControlMode = ControlMode.PercentOutput;
	protected int setpoint = RobotMap.elevatorReverseLimit;
	
	protected double elevatorPower = 0;

    public ElevatorDefaultMotionMagicAssistedRamped() {
    		requires(Robot.elevator);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    		setpoint = Robot.elevator.getCurrentPosition();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		if(Math.abs(elevatorPower) > deadband) {
    			controlMode = ControlMode.PercentOutput;
    		}else {
    			controlMode = ControlMode.MotionMagic;
    		}
    		
    		if(controlMode == ControlMode.MotionMagic && lastControlMode == ControlMode.PercentOutput) {
    			setpoint = Robot.elevator.getCurrentPosition();
    			if(setpoint < RobotMap.elevatorReverseLimit) {
    				setpoint = RobotMap.elevatorReverseLimit;
    			}else if(setpoint > RobotMap.elevatorForwardLimit - (int)(RobotMap.elevatorTraval*0.01)) {
    				setpoint = RobotMap.elevatorForwardLimit - (int)(RobotMap.elevatorTraval*0.01);
    			}
    		}
    		
    		lastControlMode = controlMode;
    		double power = Robot.m_oi.operator.getY();
		if(power > elevatorPower + rampStep) {
			elevatorPower += rampStep;
		}else if (power < elevatorPower - rampStep) {
			elevatorPower -= rampStep;
		}else {
			elevatorPower = power;
		}
    		if(controlMode == ControlMode.PercentOutput) {
    			if(Robot.elevator.getCurrentPosition() < (RobotMap.elevatorReverseLimit + (double)RobotMap.elevatorTraval*0.05) && elevatorPower < -0.25) {
    				elevatorPower = -0.25;
    			}
    			if(Robot.elevator.getCurrentPosition() > (RobotMap.elevatorForwardLimit - (double)RobotMap.elevatorTraval*0.05) && elevatorPower > 0.25) {
    				elevatorPower = 0.25;
    			}
    			Robot.elevator.setElevatorPower(elevatorPower);
    		}else {
    			Robot.elevator.moveTo(setpoint);
    		}
    		System.out.println(elevatorPower);
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
