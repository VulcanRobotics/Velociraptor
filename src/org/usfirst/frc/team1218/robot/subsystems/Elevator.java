package org.usfirst.frc.team1218.robot.subsystems;

import org.team1218.lib.ctrlSystemLogging.LoggableSRX;
import org.usfirst.frc.team1218.robot.RobotMap;
import org.usfirst.frc.team1218.robot.commands.elevator.ElevatorDefault;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Elevator extends Subsystem {
	
	static final boolean invertElevator = true;
	static final boolean invertIntake = false;

	boolean intakeStatus = false;
	LoggableSRX[] elevatorMotors = new LoggableSRX[2];
	TalonSRX[] intakeMotors = new TalonSRX[2];
	Solenoid intakeSolenoid;
	Solenoid armSolenoid;
	
	public Elevator() {
		for(int i = 0; i < 2; i ++) {
			elevatorMotors[i] = new LoggableSRX(RobotMap.elevatorMotorIds[i]);
			elevatorMotors[i].setInverted(invertElevator);
			elevatorMotors[i].enableVoltageCompensation(true);
			
			intakeMotors[i] = new TalonSRX(RobotMap.intakeMotorIds[i]);
			intakeMotors[i].setInverted(invertIntake);
			intakeMotors[i].enableVoltageCompensation(true);
		}
		
		for(int i = 1; i < 2; i++) {
			elevatorMotors[i].set(ControlMode.Follower, RobotMap.elevatorMotorIds[0]);
			intakeMotors[i].set(ControlMode.Follower, RobotMap.intakeMotorIds[0]);
		}
		
		elevatorMotors[0].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		elevatorMotors[0].setSensorPhase(false);
		
		intakeSolenoid = new Solenoid(0);
		armSolenoid = new Solenoid(1);
		
	}
	
	public void startLogging() {
		elevatorMotors[0].startLogging();
	}
	
	public void stopLogging() {
		elevatorMotors[0].stopLogging();
	}
	
	public void setElevatorPower(double elevatorPower) {
		elevatorMotors[0].set(ControlMode.PercentOutput, elevatorPower);
	}
	public void setIntakePower(double intakePower) {
		intakeMotors[0].set(ControlMode.PercentOutput, intakePower);
	}
	
	public void intakeSolenoidEngage(boolean intakeState) {
		intakeSolenoid.set(intakeState);
	}
	public void armSolenoidEngage(boolean armState) {
		armSolenoid.set(armState);
	}
	
	public void periodicTasks() {
		SmartDashboard.putString("DB/String 5", "Pe:" + elevatorMotors[0].getSelectedSensorPosition(0));
	}
	
	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new ElevatorDefault());
		
	}

}
