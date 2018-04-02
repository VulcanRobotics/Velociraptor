package org.usfirst.frc.team1218.robot.subsystems;

import org.team1218.lib.ctrlSystemLogging.LoggableSRX;
import org.usfirst.frc.team1218.robot.RobotMap;
import org.usfirst.frc.team1218.robot.commands.elevator.ElevatorDefault;
import org.usfirst.frc.team1218.robot.commands.elevator.ElevatorDefaultMotionMagic;
import org.usfirst.frc.team1218.robot.commands.elevator.ElevatorDefaultMotionMagicAssisted;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.StatusFrame;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class Elevator extends Subsystem {

	boolean isLogging = false;
	protected LoggableSRX[] elevatorMotors = new LoggableSRX[2];
	Faults elevatorFaults = new Faults();
	double elevatorPIDF[];

	public Elevator() {
		for(int i = 0; i < 2; i ++) {
			elevatorMotors[i] = new LoggableSRX(RobotMap.elevatorMotorIds[i]);
			elevatorMotors[i].setInverted(RobotMap.elevatorMotorInvert);
			elevatorMotors[i].configVoltageCompSaturation(12.0, 0);
			elevatorMotors[i].enableVoltageCompensation(true);
			elevatorMotors[i].configContinuousCurrentLimit(40, 0);
			elevatorMotors[i].configPeakCurrentLimit(50, 0);
			elevatorMotors[i].configPeakCurrentDuration(10, 0);
		}
		
		
		for(int i = 1; i < 2; i++) {
			elevatorMotors[i].set(ControlMode.Follower, RobotMap.elevatorMotorIds[0]);
		}
		
		elevatorMotors[0].config_kP(0, RobotMap.elevatorPIDF[0], 0);
		elevatorMotors[0].config_kI(0, RobotMap.elevatorPIDF[1], 0);
		elevatorMotors[0].config_kD(0, RobotMap.elevatorPIDF[2], 0);
		elevatorMotors[0].config_kF(0, RobotMap.elevatorPIDF[3], 0);
		
		elevatorMotors[0].configNominalOutputForward(0, 0);
		elevatorMotors[0].configNominalOutputReverse(0, 0);
		elevatorMotors[0].configPeakOutputForward(1, 0);
		elevatorMotors[0].configPeakOutputReverse(-1, 0);
		elevatorMotors[0].configAllowableClosedloopError(3, 0, 0);
		
		elevatorMotors[0].setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10, 0);
		elevatorMotors[0].setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10, 0);
		
		elevatorMotors[0].configMotionCruiseVelocity(RobotMap.elevatorCruiseVelocity, 0);
		elevatorMotors[0].configMotionAcceleration(RobotMap.elevatorAcceleration, 0);
	}
	
	public int getMotionMagicErr() {
		return elevatorMotors[0].getClosedLoopTarget(0) - elevatorMotors[0].getSelectedSensorPosition(0);
	}
	
	public void startLogging() {
		elevatorMotors[0].startLogging();
		isLogging = true;
	}
	
	public boolean isLogging() {
		return isLogging;
	}
	
	public void stopLogging() {
		elevatorMotors[0].stopLogging();
		isLogging = false;
	}
	
	public void setElevatorPower(double elevatorPower) {
		elevatorMotors[0].set(ControlMode.PercentOutput, elevatorPower);
	}
	
	public int getCurrentPosition() {
		return elevatorMotors[0].getSelectedSensorPosition(0);
	}
	
	public double getCurrentPosistionInches() {
		return encPosToInches(getCurrentPosition());
	}
	
	public int getTargetPosition() {
		return elevatorMotors[0].getClosedLoopTarget(0);
	}
	
	public void moveTo(int position) {
		elevatorMotors[0].set(ControlMode.MotionMagic, position);
	}
	
	public void moveToInches(double position) {
		moveTo(inchesToEncPos(position));
	}

	public void setMotionMagicSpeeds(int cruise) {
		elevatorMotors[0].configMotionCruiseVelocity(cruise, 0);
		elevatorMotors[0].configMotionAcceleration(cruise*2, 0);
	}
	
	public int inchesToEncPos(double inches) {
		return (int)((inches - RobotMap.elevatorBottomInches) * (RobotMap.elevatorTraval / RobotMap.elevatorTravalInches)) + RobotMap.elevatorReverseLimit;		
	}
	
	public double encPosToInches(int encPos) {
		return ((encPos - RobotMap.elevatorReverseLimit) * (RobotMap.elevatorTravalInches / RobotMap.elevatorTraval)) + RobotMap.elevatorBottomInches;
	}
	
	public void periodicTasks() {
		SmartDashboard.putString("DB/String 5", "Pe:" + elevatorMotors[0].getSelectedSensorPosition(0));
		SmartDashboard.putString("DB/String 6", "Ve:" + elevatorMotors[0].getSelectedSensorVelocity(0));
		if(elevatorMotors[0].getControlMode() == ControlMode.MotionMagic) {
			SmartDashboard.putString("DB/String 7", "Te:" + elevatorMotors[0].getClosedLoopTarget(0));
		}
		elevatorMotors[0].getFaults(elevatorFaults);
	}
	
	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new ElevatorDefaultMotionMagicAssisted());
		//setDefaultCommand(new ElevatorDefault());
		
	}

}
