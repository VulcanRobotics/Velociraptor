package org.usfirst.frc.team1218.robot.subsystems;

import org.usfirst.frc.team1218.robot.RobotMap;
import org.usfirst.frc.team1218.robot.commands.arm.ArmDefault;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Arm extends Subsystem {
	
	protected TalonSRX[] intakeMotors = new TalonSRX[RobotMap.intakeMotorIds.length];
	protected Solenoid armSolenoid, intakeSolenoid;
	
	public Arm() {
		for(int i = 0; i < intakeMotors.length; i++) {
			intakeMotors[i] = new TalonSRX(RobotMap.intakeMotorIds[i]);
			intakeMotors[i].setInverted(RobotMap.intakeMotorInvert[i]);
			intakeMotors[i].enableVoltageCompensation(true);
		}
		
		for(int i = 1; i < intakeMotors.length; i++) {
			intakeMotors[i].set(ControlMode.Follower, RobotMap.intakeMotorIds[0]);
		}
		
		armSolenoid = new Solenoid(RobotMap.armPort);
		intakeSolenoid = new Solenoid(RobotMap.intakePort);
	}
	
	public void setIntakePower(double intakePower) {
		intakeMotors[0].set(ControlMode.PercentOutput, intakePower+0.075);
	}
	
	public void intakeSolenoidEngage(boolean intakeState) {
		intakeSolenoid.set(intakeState);
	}
	public void armSolenoidEngage(boolean armState) {
		armSolenoid.set(!armState);
	}

	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new ArmDefault());

	}

}
