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
	static final double[] leftLowGearConstants = {0.001*1023.0/60,0,0,1023.0/1460.0};
	static final double[] rightLowGearConstants = {0.001*1023.0/60,0,0,1023.0/1400.0};
	
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
		leftMotorControllers[0].config_kP(0, leftLowGearConstants[kp], 10);
		leftMotorControllers[0].config_kI(0, leftLowGearConstants[ki], 10);
		leftMotorControllers[0].config_kD(0, leftLowGearConstants[kd], 10);
		leftMotorControllers[0].config_kF(0, leftLowGearConstants[kf], 10);
		
		rightMotorControllers[0].config_kP(0, rightLowGearConstants[kp], 0);
		rightMotorControllers[0].config_kI(0, rightLowGearConstants[ki], 0);
		rightMotorControllers[0].config_kD(0, rightLowGearConstants[kd], 0);
		rightMotorControllers[0].config_kF(0, rightLowGearConstants[kf], 0);
		
		leftMotorControllers[0].configNominalOutputForward(0, 0);
		leftMotorControllers[0].configPeakOutputForward(1, 0);
		leftMotorControllers[0].configNominalOutputReverse(0, 0);
		leftMotorControllers[0].configPeakOutputReverse(-1, 0);
		
		rightMotorControllers[0].configNominalOutputForward(0, 0);
		rightMotorControllers[0].configPeakOutputForward(1, 0);
		rightMotorControllers[0].configNominalOutputReverse(0, 0);
		rightMotorControllers[0].configPeakOutputReverse(-1, 0);
		
		shifter = new Solenoid(shifterPort);
	}
	
	public void setPower(double leftPower, double rightPower) {
		leftPower = clampPower(leftPower);
		rightPower = clampPower(rightPower);
		leftMotorControllers[0].set(ControlMode.PercentOutput, leftPower);
		rightMotorControllers[0].set(ControlMode.PercentOutput, rightPower);
	}
	
	/**
	 * turns on velocity closed-loop. sets target velocity for left and right.
	 * @param leftVelocity in encoder counts per 100ms
	 * @param rightVelocity in encoder counts per 100ms
	 */
	public void setVelocity(int leftVelocity, int rightVelocity) {
		leftMotorControllers[0].set(ControlMode.Velocity, leftVelocity);
		rightMotorControllers[0].set(ControlMode.Velocity, rightVelocity);

	}
	
	protected double clampPower(double power) {
		return Math.max(-1.0, Math.min(1.0, power));
	}
	
	public void periodicTasks() {
		//publish left and right encoder Position to Dashboard.
		SmartDashboard.putString("DB/String 0", "Pl:" + leftMotorControllers[0].getSelectedSensorPosition(0));
		SmartDashboard.putString("DB/String 1", "Pr:" + rightMotorControllers[0].getSelectedSensorPosition(0));
		SmartDashboard.putString("DB/String 2", "Vl:" + leftMotorControllers[0].getSelectedSensorVelocity(0));
		SmartDashboard.putString("DB/String 3", "Vr:" + rightMotorControllers[0].getSelectedSensorVelocity(0));
		SmartDashboard.putString("DB/String 5", "El:" + leftMotorControllers[0].getClosedLoopError(0));
		SmartDashboard.putString("DB/String 6", "Er:" + rightMotorControllers[0].getClosedLoopError(0));
		SmartDashboard.putString("DB/String 7", "Pl:" + leftMotorControllers[0].getMotorOutputVoltage());
		SmartDashboard.putString("DB/String 8", "Pr:" + rightMotorControllers[0].getMotorOutputVoltage());
	}
	

    public void initDefaultCommand() {
        setDefaultCommand(new DriveDefault());
    }
}

