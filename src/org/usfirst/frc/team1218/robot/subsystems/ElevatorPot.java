package org.usfirst.frc.team1218.robot.subsystems;

import org.usfirst.frc.team1218.robot.RobotMap;
import org.usfirst.frc.team1218.robot.commands.elevator.ElevatorDefault;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;

public class ElevatorPot extends Elevator {
	
	public static final int elevatorReverseLimit = 10;
	public static final int elevatorForwardLimit = 1020;
	public static final int elevatorCruiserVelocity = 50;
	public static final int elevatorAcceleration = 100;
	public static final double[] elevatorPIDF = {0.0,0.0,0.0,0.0};
	
	public ElevatorPot() {
		super();
		
		elevatorMotors[0].configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0); //select Analog input as feedback device
		elevatorMotors[0].setSensorPhase(false);
		elevatorMotors[0].configReverseSoftLimitThreshold(elevatorReverseLimit, 0);
		elevatorMotors[0].configReverseSoftLimitEnable(true,0);
		elevatorMotors[0].configForwardSoftLimitThreshold(elevatorForwardLimit, 0);
		elevatorMotors[0].configForwardSoftLimitEnable(true,0);
		
		elevatorMotors[0].config_kP(0, elevatorPIDF[0], 0);
		elevatorMotors[0].config_kI(0, elevatorPIDF[1], 0);
		elevatorMotors[0].config_kD(0, elevatorPIDF[2], 0);
		elevatorMotors[0].config_kF(0, elevatorPIDF[3], 0);
		
		elevatorMotors[0].setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10, 0);
		elevatorMotors[0].setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10, 0);
		
		elevatorMotors[0].configMotionCruiseVelocity(RobotMap.elevatorCruiseVelocity, 0);
		elevatorMotors[0].configMotionAcceleration(RobotMap.elevatorAcceleration, 0);
		
	}
	
	@Override
	protected void initDefaultCommand(){
		setDefaultCommand(new ElevatorDefault());
	}
}
