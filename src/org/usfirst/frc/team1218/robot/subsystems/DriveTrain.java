package org.usfirst.frc.team1218.robot.subsystems;

import org.usfirst.frc.team1218.robot.commands.driveTrain.DriveDefault;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTrain extends Subsystem {
	
	TalonSRX[] leftMotorControllers = new TalonSRX[3];
	TalonSRX[] rightMotorControllers = new TalonSRX[3];
	Solenoid shifter;
	
	public DriveTrain(int[] leftMotorControllerIds, int[] rightMotorControllerIds, boolean invertLeft, boolean invertRight,int shifterPort) {
		for(int i = 0; i < 3; i++) {
			leftMotorControllers[i] = new TalonSRX(leftMotorControllerIds[i]);
			leftMotorControllers[i].setInverted(invertLeft);
			leftMotorControllers[i].enableVoltageCompensation(true);
			
			rightMotorControllers[i] = new TalonSRX(rightMotorControllerIds[i]);
			rightMotorControllers[i].setInverted(invertRight);
			rightMotorControllers[i].enableVoltageCompensation(true);
		}
		for(int i = 1; i < 3; i++) {
			leftMotorControllers[i].set(ControlMode.Follower, leftMotorControllerIds[0]);
			
			rightMotorControllers[i].set(ControlMode.Follower, rightMotorControllerIds[0]);
		}
		//setting up encoder feedback on Master Controllers
		//encoder is set as feed back device for PID loop 0(the Main loop)
		//configSelectedFeedbackSensor(feedbackDevice,loop#,timeout)
		leftMotorControllers[0].configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		rightMotorControllers[0].configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0, 0);
		shifter = new Solenoid(shifterPort);
	}
	
	public void setPower(double leftPower, double rightPower) {
		leftMotorControllers[0].set(ControlMode.PercentOutput, leftPower);
		
		rightMotorControllers[0].set(ControlMode.PercentOutput, rightPower);
	}
	
	public void periodicTasks() {
		//publish left and right encoder Position to Dashboard.
		SmartDashboard.putString("DB/String 0", "" + leftMotorControllers[0].getSelectedSensorPosition(0));
		SmartDashboard.putString("DB/String 1", "" + rightMotorControllers[0].getSelectedSensorPosition(0));
	}
	

    public void initDefaultCommand() {
        setDefaultCommand(new DriveDefault());
    }
}

