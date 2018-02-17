package org.usfirst.frc.team1218.robot.subsystems;

import org.team1218.lib.ctrlSystemLogging.LoggableSRX;
import org.usfirst.frc.team1218.robot.RobotMap;
import org.usfirst.frc.team1218.robot.commands.driveTrain.DriveDefault;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;

/**
 *
 */
public class DriveTrain extends Subsystem {
//	public static final int kp = 0;
//	public static final int ki = 1;
//	public static final int kd = 2;
//	public static final int kf = 3;
	//low gear pid constants
//	static final double[] leftLowGearConstants = {1.05,0,0,0.75};
//	static final double[] rightLowGearConstants = {1.05,0,0,0.75};
	
//	public static final int MaxSpeed = 1400;		// encoder ticks per 100ms
	public static final double wheelDiameterInches = 4.0;
//	public static final int encTicksPerRev = 2000;
//	public static final double trackWidthInches = 28.0;
	
	/**
	 * Return motor velocity (in encoder counts per 100ms) for a given robot velocity (in ft per sec)
	 * @param ftPerSec robot velocity in ft/sec
	 */
	public static int ftPerSecToEncVel(double ftPerSec) {
		// (ftPerSec / ftPerRev) = revPerSec
		// revPerSec * ticksPerRev = ticksPerSec
		// ticksPerSec / 10 = ticksPer100ms = encVel
		return (int)(((ftPerSec / (wheelDiameterInches * Math.PI / 12.0)) * RobotMap.encTicksPerRev) / 10.0);
	}
	
	public static double radiansToInches(double angleInRadians) {
		return ((RobotMap.trackWidthInches / 2.0) * angleInRadians);
	}
	
	LoggableSRX[] leftMotorControllers = new LoggableSRX[3];
	LoggableSRX[] rightMotorControllers = new LoggableSRX[3];
	Solenoid shifter;
	Solenoid pto;
	AHRS navx;
	boolean enableLogging = true;
	boolean isLogging = false;
	
	

	public DriveTrain() {
		for(int i = 0; i < 3; i++) {
			leftMotorControllers[i] = new LoggableSRX(RobotMap.leftMotorControllerIds[i]);
			leftMotorControllers[i].setInverted(RobotMap.leftInverted);
			leftMotorControllers[i].enableVoltageCompensation(true);
			leftMotorControllers[i].configOpenloopRamp(0.25, 0);
			
			rightMotorControllers[i] = new LoggableSRX(RobotMap.rightMotorControllerIds[i]);
			rightMotorControllers[i].setInverted(RobotMap.rightInverted);
			rightMotorControllers[i].enableVoltageCompensation(true);
			rightMotorControllers[i].configOpenloopRamp(0.25, 0);
		}
		for(int i = 1; i < 3; i++) {
			leftMotorControllers[i].set(ControlMode.Follower, RobotMap.leftMotorControllerIds[0]);
			
			rightMotorControllers[i].set(ControlMode.Follower, RobotMap.rightMotorControllerIds[0]);
		}
		
		navx = new AHRS(I2C.Port.kMXP);
		
		//setting up encoder feedback on Master Controllers
		//encoder is set as feed back device for PID loop 0(the Main loop)
		//configSelectedFeedbackSensor(feedbackDevice,loop#,timeout)
		leftMotorControllers[0].configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		leftMotorControllers[0].setSensorPhase(true);
		rightMotorControllers[0].configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0, 0);
		rightMotorControllers[0].setSensorPhase(true);
		//load pid constants
		leftMotorControllers[0].config_kP(0, RobotMap.leftLowGearPIDF[0], 10);
		leftMotorControllers[0].config_kI(0, RobotMap.leftLowGearPIDF[1], 10);
		leftMotorControllers[0].config_kD(0, RobotMap.leftLowGearPIDF[2], 10);
		leftMotorControllers[0].config_kF(0, RobotMap.leftLowGearPIDF[3], 10);
		
		rightMotorControllers[0].config_kP(0, RobotMap.rightLowGearPIDF[0], 0);
		rightMotorControllers[0].config_kI(0, RobotMap.rightLowGearPIDF[1], 0);
		rightMotorControllers[0].config_kD(0, RobotMap.rightLowGearPIDF[2], 0);
		rightMotorControllers[0].config_kF(0, RobotMap.rightLowGearPIDF[3], 0);
		
		leftMotorControllers[0].configNominalOutputForward(0, 0);
		leftMotorControllers[0].configPeakOutputForward(1, 0);
		leftMotorControllers[0].configNominalOutputReverse(0, 0);
		leftMotorControllers[0].configPeakOutputReverse(-1, 0);
		
		rightMotorControllers[0].configNominalOutputForward(0, 0);
		rightMotorControllers[0].configPeakOutputForward(1, 0);
		rightMotorControllers[0].configNominalOutputReverse(0, 0);
		rightMotorControllers[0].configPeakOutputReverse(-1, 0);
		
		shifter = new Solenoid(RobotMap.shifterPort);
		pto = new Solenoid(RobotMap.ptoPort);
		engagePto(false);
	}
	
	/**
	 * turns on velocity closed-loop. sets target velocity for left and right.
	 * @param leftVelocity in encoder counts per 100ms
	 * @param rightVelocity in encoder counts per 100ms
	 */
	
	public void setPower(double leftPower, double rightPower) {
		leftPower = clampPower(leftPower);
		rightPower = clampPower(rightPower);
		leftMotorControllers[0].set(ControlMode.PercentOutput, leftPower);
		rightMotorControllers[0].set(ControlMode.PercentOutput, rightPower);
	}
	
	/**
	 * drives using velocity closed-loop, with target velocity as encoder counts per 100ms.
	 * @param leftVelocity in encoder counts per 100ms
	 * @param rightVelocity in encoder counts per 100ms
	 */
	public void setVelocity(int leftVelocity, int rightVelocity) {
		leftMotorControllers[0].set(ControlMode.Velocity, leftVelocity);
		rightMotorControllers[0].set(ControlMode.Velocity, rightVelocity);
	}

	public void setBrake(NeutralMode mode) {
		for(int i = 0; i < leftMotorControllers.length;i++) {
			leftMotorControllers[i].setNeutralMode(mode);
			rightMotorControllers[i].setNeutralMode(mode);
		}
	}
	/**
	 * drives using velocity closed-loop, with target velocity as % output.
	 * @param leftPower in % output, -1.0 to 1.0
	 */
	public void setVelocity(double leftPower, double rightPower) {
		leftMotorControllers[0].set(ControlMode.Velocity, leftPower*RobotMap.lowGearMaxSpeed);
		rightMotorControllers[0].set(ControlMode.Velocity, rightPower*RobotMap.lowGearMaxSpeed);
	}
	
	protected double clampPower(double power) {
		return Math.max(-1.0, Math.min(1.0, power));
	}
	
	public void startLogging() {
		if (enableLogging) {
			isLogging = true;
			leftMotorControllers[0].startLogging();
			rightMotorControllers[0].startLogging();
		}
	}
	
	public void stopLogging() {
		isLogging = false;
		leftMotorControllers[0].stopLogging();
		rightMotorControllers[0].stopLogging();
	}
	
	public boolean isLogging() {
		return isLogging;
	}
	
	public boolean isLoggingEnabled() {
		return enableLogging;
	}

	public void setEnableLogging(boolean enableLogging) {
		this.enableLogging = enableLogging;
	}
	
	public void shift(boolean shift) {
		shifter.set(shift);
	}
	
	public void engagePto(boolean engage) {
		pto.set(!engage);
	}
	
	public boolean isPtoEngaged() {
		return !pto.get();
	}
	public double getHeading() {
		return navx.getAngle();
	}
	
	public void periodicTasks() {
		//publish left and right encoder Position to Dashboard.
		SmartDashboard.putString("DB/String 0", "Pl:" + leftMotorControllers[0].getSelectedSensorPosition(0));
		SmartDashboard.putString("DB/String 1", "Pr:" + rightMotorControllers[0].getSelectedSensorPosition(0));
		SmartDashboard.putString("DB/String 2", "Vl:" + leftMotorControllers[0].getSelectedSensorVelocity(0));
		SmartDashboard.putString("DB/String 3", "Vr:" + rightMotorControllers[0].getSelectedSensorVelocity(0));
		SmartDashboard.putString("DB/String 4", "H" + getHeading());
	}
	

    public void initDefaultCommand() {
       setDefaultCommand(new DriveDefault());
    }
    
    public void processMotionProfileBuffer() {
    	leftMotorControllers[0].processMotionProfileBuffer();
    	rightMotorControllers[0].processMotionProfileBuffer();
    }
}

