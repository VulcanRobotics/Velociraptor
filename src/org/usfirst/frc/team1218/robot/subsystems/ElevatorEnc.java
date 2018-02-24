package org.usfirst.frc.team1218.robot.subsystems;


import org.usfirst.frc.team1218.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;


import edu.wpi.first.wpilibj.Notifier;


public class ElevatorEnc extends Elevator {
	
	//static final boolean invertElevator = false;
	//static final boolean[] invertIntake = {true,false};

	boolean intakeStatus = false;
	private Notifier processMPBuffer = new Notifier(new Runnable() {

		@Override
		public void run() {
			elevatorMotors[0].processMotionProfileBuffer();
	
		}
		
	});
	
	public ElevatorEnc() {
		super();
		
		elevatorMotors[0].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		elevatorMotors[0].setSensorPhase(false);
		elevatorMotors[0].configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
		elevatorMotors[0].configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
		elevatorMotors[0].configForwardSoftLimitThreshold(290000, 0);
		elevatorMotors[0].configForwardSoftLimitEnable(true, 0);
		
		elevatorMotors[0].config_kP(0, RobotMap.elevatorPIDF[0], 0);
		elevatorMotors[0].config_kI(0, RobotMap.elevatorPIDF[1], 0);
		elevatorMotors[0].config_kD(0, RobotMap.elevatorPIDF[2], 0);
		elevatorMotors[0].config_kF(0, RobotMap.elevatorPIDF[3], 0);
		
		elevatorMotors[0].setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10, 0);
		elevatorMotors[0].setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10, 0);
		
		elevatorMotors[0].configMotionCruiseVelocity(RobotMap.elevatorCruiseVelocity, 0);
		elevatorMotors[0].configMotionAcceleration(RobotMap.elevatorAcceleration, 0);
	}
	
	@Override
	public void periodicTasks(){
		super.periodicTasks();
		elevatorMotors[0].getFaults(elevatorFaults);
		if(elevatorFaults.ReverseLimitSwitch == true) {
			elevatorMotors[0].setSelectedSensorPosition(0, 0, 0);
		}
	}

}
