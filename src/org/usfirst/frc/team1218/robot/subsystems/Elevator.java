package org.usfirst.frc.team1218.robot.subsystems;

import org.team1218.lib.ctrlSystemLogging.LoggableSRX;
import org.usfirst.frc.team1218.robot.RobotMap;
import org.usfirst.frc.team1218.robot.commands.elevator.ElevatorDefaultMotionMagic;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class Elevator extends Subsystem {

	boolean isLogging = false;
	protected LoggableSRX[] elevatorMotors = new LoggableSRX[2];
	Faults elevatorFaults = new Faults();

	public Elevator() {
		for(int i = 0; i < 2; i ++) {
			elevatorMotors[i] = new LoggableSRX(RobotMap.elevatorMotorIds[i]);
			elevatorMotors[i].setInverted(RobotMap.elevatorMotorInvert);
			elevatorMotors[i].enableVoltageCompensation(true);
			elevatorMotors[i].configContinuousCurrentLimit(15, 0);
			elevatorMotors[i].configPeakCurrentLimit(30, 0);
			elevatorMotors[i].configPeakCurrentDuration(250, 0);
		}
		
		
		for(int i = 1; i < 2; i++) {
			elevatorMotors[i].set(ControlMode.Follower, RobotMap.elevatorMotorIds[0]);
		}
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

	public void moveTo(int position) {
		System.out.println("moving to " + position);
		elevatorMotors[0].set(ControlMode.MotionMagic, position);
	}

	public void periodicTasks() {
		SmartDashboard.putString("DB/String 5", "Pe:" + elevatorMotors[0].getSelectedSensorPosition(0));
		SmartDashboard.putString("DB/String 6", "Ve:" + elevatorMotors[0].getSelectedSensorVelocity(0));
		elevatorMotors[0].getFaults(elevatorFaults);
	}

	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new ElevatorDefaultMotionMagic());
		
	}

}