package org.usfirst.frc.team1218.robot.subsystems;

import org.usfirst.frc.team1218.robot.commands.driveTrain.DriveDefault;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DriveTrain extends Subsystem {
	
	TalonSRX[] leftMotorControllers;
	TalonSRX[] rightMotorControllers;
	
	public DriveTrain(int[] leftMotorControllerIds, int[] rightMotorControllerIds, boolean invertLeft, boolean invertRight) {
		for(int i = 0; i < 3; i++) {
			leftMotorControllers[i] = new TalonSRX(leftMotorControllerIds[i]);
			leftMotorControllers[i].setInverted(invertLeft);
			leftMotorControllers[i].enableVoltageCompensation(true);
			
			rightMotorControllers[i] = new TalonSRX(rightMotorControllerIds[i]);
			rightMotorControllers[i].setInverted(invertRight);
			rightMotorControllers[i].enableVoltageCompensation(true);
		}
	}
	
	public void setPower(double leftPower, double rightPower) {
		for(TalonSRX leftMotor: leftMotorControllers) {
			leftMotor.set(ControlMode.PercentOutput, leftPower);
		}
		
		for(TalonSRX rightMotor: rightMotorControllers) {
			rightMotor.set(ControlMode.PercentOutput, rightPower);
		}
	}
	
	

    public void initDefaultCommand() {
        setDefaultCommand(new DriveDefault());
    }
}

