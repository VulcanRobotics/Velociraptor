package org.usfirst.frc.team1218.robot.subsystems;

import org.usfirst.frc.team1218.robot.LoggableSRX;
import org.usfirst.frc.team1218.robot.commands.driveTrain.DriveDefault;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;


public class Elevator extends Subsystem {
	
	static final boolean invertElevator = false;
	static final boolean invertIntake = false;

	boolean intakeStatus = false;
	TalonSRX[] elevatorMotors = new TalonSRX[2];
	TalonSRX[] intakeMotors = new TalonSRX[2];
	Solenoid intakeSolenoid;
	Solenoid armSolenoid;
	
	public Elevator(int[] elevatorMotorIds, int[] intakeMotorIds) {
		for(int i = 0; i < 2; i ++) {
			elevatorMotors[i] = new TalonSRX(elevatorMotorIds[i]);
			elevatorMotors[i].setInverted(invertElevator);
			elevatorMotors[i].enableVoltageCompensation(true);
			
			intakeMotors[i] = new TalonSRX(intakeMotorIds[i]);
			intakeMotors[i].setInverted(invertIntake);
			intakeMotors[i].enableVoltageCompensation(true);
		}
		
		for(int i = 1; i < 2; i++) {
			elevatorMotors[i].set(ControlMode.Follower, elevatorMotorIds[0]);
			intakeMotors[i].set(ControlMode.Follower, elevatorMotorIds[0]);
		}
		
		intakeSolenoid = new Solenoid(0);
		armSolenoid = new Solenoid(1);
		
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
	
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}

}
