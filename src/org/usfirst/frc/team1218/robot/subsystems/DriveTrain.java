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
	public static final int kp = 0;
	public static final int ki = 1;
	public static final int kd = 2;
	public static final int kf = 3;
	//low gear pid constants
	static final double[] leftLowGearConstants = {0,0,0,1.0/1460.0};
	static final double[] rightLowGearConstants = {0,0,0,1.0/1400.0};
	
	static final int MaxSpeed = 1300;
	
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
		leftMotorControllers[0].setSensorPhase(true);
		rightMotorControllers[0].configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0, 0);
		rightMotorControllers[0].setSensorPhase(true);
		//load pid constants
		leftMotorControllers[0].config_kP(0, leftLowGearConstants[kp], 0);
		leftMotorControllers[0].config_kI(0, leftLowGearConstants[ki], 0);
		leftMotorControllers[0].config_kD(0, leftLowGearConstants[kd], 0);
		leftMotorControllers[0].config_kF(0, leftLowGearConstants[kf], 0);
		
		rightMotorControllers[0].config_kP(0, rightLowGearConstants[kp], 0);
		rightMotorControllers[0].config_kI(0, rightLowGearConstants[ki], 0);
		rightMotorControllers[0].config_kD(0, rightLowGearConstants[kd], 0);
		rightMotorControllers[0].config_kF(0, rightLowGearConstants[kf], 0);
		
		shifter = new Solenoid(shifterPort);
	}
	
	public void setPower(double leftPower, double rightPower) {
		leftMotorControllers[0].set(ControlMode.Velocity, leftPower*MaxSpeed);
		
		rightMotorControllers[0].set(ControlMode.Velocity, rightPower*MaxSpeed);
	}
	
	public void periodicTasks() {
		//publish left and right encoder Position to Dashboard.
		SmartDashboard.putString("DB/String 0", "Pl:" + leftMotorControllers[0].getSelectedSensorPosition(0));
		SmartDashboard.putString("DB/String 1", "Pr:" + rightMotorControllers[0].getSelectedSensorPosition(0));
		SmartDashboard.putString("DB/String 2", "Vl:" + leftMotorControllers[0].getSelectedSensorVelocity(0));
		SmartDashboard.putString("DB/String 3", "Vr:" + rightMotorControllers[0].getSelectedSensorVelocity(0));
	}
	

    public void initDefaultCommand() {
        setDefaultCommand(new DriveDefault());
    }
}

