package org.usfirst.frc.team1218.robot.subsystems;

import org.team1218.lib.ctrlSystemLogging.LoggableSRX;
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
		super();
	}

	public Elevator(String name) {
		super(name);
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
		if(elevatorFaults.ReverseLimitSwitch == true) {
			elevatorMotors[0].setSelectedSensorPosition(0, 0, 0);
		}
	}

	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new ElevatorDefaultMotionMagic());
		
	}

}