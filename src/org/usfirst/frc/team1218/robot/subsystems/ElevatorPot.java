package org.usfirst.frc.team1218.robot.subsystems;

import org.usfirst.frc.team1218.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class ElevatorPot extends Elevator {
	
	public ElevatorPot() {
		super();
		
		elevatorMotors[0].configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0); //select Analog input as feedback device
		elevatorMotors[0].setSensorPhase(false);
		elevatorMotors[0].configReverseSoftLimitThreshold(RobotMap.elevatorReverseLimit, 0);
		elevatorMotors[0].configReverseSoftLimitEnable(true,0);
		elevatorMotors[0].configForwardSoftLimitThreshold(RobotMap.elevatorForwardLimit, 0);
		elevatorMotors[0].configForwardSoftLimitEnable(true,0);
		
	}
	
}
