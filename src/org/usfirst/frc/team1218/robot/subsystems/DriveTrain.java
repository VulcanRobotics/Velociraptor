package org.usfirst.frc.team1218.robot.subsystems;
import org.team1218.lib.ctrlSystemLogging.LoggableSRX;
import org.usfirst.frc.team1218.robot.RobotMap;
import org.usfirst.frc.team1218.robot.commands.driveTrain.DriveDefault;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.kauailabs.navx.frc.AHRS;
import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.Trajectory.Segment;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;

/**
 *
 */
public class DriveTrain extends Subsystem {
	
	public static final double wheelDiameterInches = 4.0;
	
	/**
	 * Return motor velocity (in encoder counts per 100ms) for a given robot velocity (in ft per sec)
	 * @param ftPerSec robot velocity in ft/sec
	 */	
	protected static double segmentToFFVoltage(Segment s, double Kv, double Ka, double VInter) {
		double v = Kv * s.vel;
		if (v < 0) {
			return (v + (Ka * s.acc) - VInter);
		} else {
			return (v + (Ka * s.acc) + VInter);
		}
	}
	
	public static int ftPerSecToEncVel(double ftPerSec) {
		int encVel = (int)(((ftPerSec / (wheelDiameterInches * Math.PI / 12.0)) * RobotMap.encTicksPerRev) / 10.0);
//		System.out.println("ft/sec: " + ftPerSec + " envVel: " + encVel);
		return encVel;
				
	}
	
	public static int ftToEncPos(double ft) {
		return (int)((ft/(wheelDiameterInches * Math.PI / 12.0))*RobotMap.encTicksPerRev);
	}
	
	public static double encVelToFTPerSec(int encVel) {
		return (((encVel*10.0)/RobotMap.encTicksPerRev)*(wheelDiameterInches * Math.PI / 12.0));
	}
	
	public static double encPostoFt(int encPos) {
		return (((double)encPos/RobotMap.encTicksPerRev)*(wheelDiameterInches * Math.PI / 12.0));
	}
	
	public static double radiansToInches(double angleInRadians) {
		return ((RobotMap.trackWidthInches / 2.0) * angleInRadians);
	}
	
	LoggableSRX[] leftMotorControllers = new LoggableSRX[3];
	LoggableSRX[] rightMotorControllers = new LoggableSRX[3];
	Solenoid shifter;
	Solenoid pto;
	public AHRS navx;
	
	MotionProfileStatus leftStat = new MotionProfileStatus();
	MotionProfileStatus rightStat = new MotionProfileStatus();
	
	boolean enableLogging = true;
	boolean isLogging = false;
	boolean isPathFollowing = false;

	public DriveTrain() {
		for(int i = 0; i < 3; i++) {
			leftMotorControllers[i] = new LoggableSRX(RobotMap.leftMotorControllerIds[i]);
			leftMotorControllers[i].setInverted(RobotMap.leftInverted);
			leftMotorControllers[i].configVoltageCompSaturation(12.0, 0);
			leftMotorControllers[i].enableVoltageCompensation(true);
			leftMotorControllers[i].configOpenloopRamp(0.25, 0);
			
			rightMotorControllers[i] = new LoggableSRX(RobotMap.rightMotorControllerIds[i]);
			rightMotorControllers[i].setInverted(RobotMap.rightInverted);
			rightMotorControllers[i].configVoltageCompSaturation(12.0, 0);
			rightMotorControllers[i].enableVoltageCompensation(true);
			rightMotorControllers[i].configOpenloopRamp(0.25, 0);
			
			leftMotorControllers[i].setNeutralMode(NeutralMode.Coast);
			rightMotorControllers[i].setNeutralMode(NeutralMode.Coast);
		}
		for(int i = 1; i < 3; i++) {
			leftMotorControllers[i].set(ControlMode.Follower, RobotMap.leftMotorControllerIds[0]);
			
			rightMotorControllers[i].set(ControlMode.Follower, RobotMap.rightMotorControllerIds[0]);
		}
		
		navx = new AHRS(I2C.Port.kOnboard);
		
		//setting up encoder feedback on Master Controllers
		//encoder is set as feed back device for PID loop 0(the Main loop)
		//configSelectedFeedbackSensor(feedbackDevice,loop#,timeout)
		leftMotorControllers[0].configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		leftMotorControllers[0].setSensorPhase(true);
		rightMotorControllers[0].configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0, 0);
		rightMotorControllers[0].setSensorPhase(true);
		//load pid constants
		//loadPIDFConstants(RobotMap.leftLowGearPIDF,RobotMap.rightLowGearPIDF);
		
		leftMotorControllers[0].configNominalOutputForward(0, 0);
		leftMotorControllers[0].configPeakOutputForward(1, 0);
		leftMotorControllers[0].configNominalOutputReverse(0, 0);
		leftMotorControllers[0].configPeakOutputReverse(-1, 0);
		
		rightMotorControllers[0].configNominalOutputForward(0, 0);
		rightMotorControllers[0].configPeakOutputForward(1, 0);
		rightMotorControllers[0].configNominalOutputReverse(0, 0);
		rightMotorControllers[0].configPeakOutputReverse(-1, 0);
		
		leftMotorControllers[0].configMotionCruiseVelocity(7900, 0);
		leftMotorControllers[0].configMotionAcceleration(7900, 0);
		rightMotorControllers[0].configMotionCruiseVelocity(7900, 0);
		rightMotorControllers[0].configMotionAcceleration(7900, 0);
		
		shifter = new Solenoid(RobotMap.shifterPort);
		pto = new Solenoid(RobotMap.ptoPort);
		engagePto(false);
		System.out.println("DriveTrain: leftInverted="+leftMotorControllers[0].getInverted());
		System.out.println("DriveTrain: rightInverted="+rightMotorControllers[0].getInverted());
	}
	
	public void loadPIDFConstants(double[] leftPIDF,double[] rightPIDF) {
		leftMotorControllers[0].config_kP(0, leftPIDF[0], 0);
		leftMotorControllers[0].config_kI(0, leftPIDF[1], 0);
		leftMotorControllers[0].config_kD(0, leftPIDF[2], 0);
		leftMotorControllers[0].config_kF(0, leftPIDF[3], 0);
		
		rightMotorControllers[0].config_kP(0, rightPIDF[0], 0);
		rightMotorControllers[0].config_kI(0, rightPIDF[1], 0);
		rightMotorControllers[0].config_kD(0, rightPIDF[2], 0);
		rightMotorControllers[0].config_kF(0, rightPIDF[3], 0);
	}
		
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
	
	public void zeroPos() {
		leftMotorControllers[0].setSelectedSensorPosition(0, 0, 0);
		rightMotorControllers[0].setSelectedSensorPosition(0, 0, 0);
	}
	
	/**
	 * drives using velocity closed-loop, with target velocity as % output.
	 * @param leftPower in % output, -1.0 to 1.0
	 */
	public void setVelocity(double leftPower, double rightPower) {
		if(leftMotorControllers[0].getControlMode() != ControlMode.Velocity || rightMotorControllers[0].getControlMode() != ControlMode.Velocity) {
			loadPIDFConstants(RobotMap.leftLowGearPIDF,RobotMap.rightLowGearPIDF);
		}
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
		pto.set(engage);
	}
	
	public boolean isPtoEngaged() {
		return pto.get();
	}
	
	public double getHeading() {
		return navx.getAngle();
	}
	
	public boolean isPathFollowing() {
		return isPathFollowing;
	}

	public void setPathFollowing(boolean isPathFollowing) {
		this.isPathFollowing = isPathFollowing;
	}
	
	public void setPath(Path path, double period) {
		//clear any outstanding motion profile points
		leftMotorControllers[0].clearMotionProfileTrajectories();
		rightMotorControllers[0].clearMotionProfileTrajectories();
		
		//set period
		leftMotorControllers[0].configMotionProfileTrajectoryPeriod(0, 0);
		rightMotorControllers[0].configMotionProfileTrajectoryPeriod(0, 0);
		
		Trajectory leftTrajectory, rightTrajectory;
		leftTrajectory = path.getLeftWheelTrajectory();
		rightTrajectory = path.getRightWheelTrajectory();
		
		//point variable
		TrajectoryPoint point = new TrajectoryPoint();
		
		for(int i = 0; i < leftTrajectory.getNumSegments(); i++) {
			point.position = ftToEncPos(leftTrajectory.getSegment(i).pos);
			point.velocity = ftPerSecToEncVel(leftTrajectory.getSegment(i).vel);//segmentToFFVoltage(leftTrajectory.getSegment(i), RobotMap.leftLowGearKv, 
							//RobotMap.leftLowGearKa*1.2, RobotMap.leftLowGearVInter); //ftPerSecToEncVel(leftTrajectory.getSegment(i).vel);
			System.out.println("left Point " + i + "origPos: " + leftTrajectory.getSegment(i).pos + 
								" origVel: " + leftTrajectory.getSegment(i).vel + " pos: " + point.position + 
								" vel: " + point.velocity);
			point.headingDeg = 0;
			point.profileSlotSelect0 = 0;
			point.profileSlotSelect1 = 0;
			point.timeDur = TrajectoryPoint.TrajectoryDuration.Trajectory_Duration_100ms;
			if(i == 0) {
				point.zeroPos = true;
			}else {
				point.zeroPos = false;
			}
			if(i == leftTrajectory.getNumSegments() -1) {
				point.isLastPoint = true;
			}else {
				point.isLastPoint = false;
			}
			
			leftMotorControllers[0].pushMotionProfileTrajectory(point);
		}
		
		for(int i = 0; i < rightTrajectory.getNumSegments(); i++) {
			point.position = ftToEncPos(rightTrajectory.getSegment(i).pos);
			point.velocity = ftPerSecToEncVel(rightTrajectory.getSegment(i).vel);//segmentToFFVoltage(rightTrajectory.getSegment(i), RobotMap.rightLowGearKv,
							//RobotMap.rightLowGearKa*1.2, RobotMap.rightLowGearVInter);
					//ftPerSecToEncVel(rightTrajectory.getSegment(i).vel);
			point.headingDeg = 0;
			point.profileSlotSelect0 = 0;
			point.profileSlotSelect1 = 0;
			point.timeDur = TrajectoryPoint.TrajectoryDuration.Trajectory_Duration_100ms;
			if(i == 0) {
				point.zeroPos = true;
			}else {
				point.zeroPos = false;
			}
			if(i == rightTrajectory.getNumSegments() -1) {
				point.isLastPoint = true;
			}else {
				point.isLastPoint = false;
			}
			
			rightMotorControllers[0].pushMotionProfileTrajectory(point);
		}	
		
	}
	
	public void startPath() {
		loadPIDFConstants(RobotMap.leftLowGearTalonMPPIDF, RobotMap.rightLowGearTalonMPPIDF); //RobotMap.leftLowGearTalonMPPIDF,RobotMap.rightLowGearTalonMPPIDF);
		setBrake(NeutralMode.Coast);
		leftMotorControllers[0].setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 5, 0);
		rightMotorControllers[0].setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 5, 0);
		leftMotorControllers[0].changeMotionControlFramePeriod(5);
		rightMotorControllers[0].changeMotionControlFramePeriod(5);
		leftMotorControllers[0].set(ControlMode.MotionProfile,SetValueMotionProfile.Enable.value);
		rightMotorControllers[0].set(ControlMode.MotionProfile,SetValueMotionProfile.Enable.value);
		if(enableLogging) {
			startLogging();
		}
		isPathFollowing = true;
	}
	public void turnMotionMagic(double angle) {
		double distance = angle*RobotMap.trackWidthInches/12.0;
		moveMotionMagic(-distance,distance);
	}
	
	public void moveMotionMagic(double leftFt,double rightFt) {
		moveMotionMagic(ftToEncPos(leftFt),ftToEncPos(rightFt));
	}
	
	public void moveMotionMagic(int leftEncCounts,int rightEncCounts) {
		leftMotorControllers[0].set(ControlMode.MotionMagic, leftMotorControllers[0].getSelectedSensorPosition(0) + leftEncCounts);
		rightMotorControllers[0].set(ControlMode.MotionMagic, rightMotorControllers[0].getSelectedSensorPosition(0) + rightEncCounts);
	}
	
	public boolean motionMagicOnTarget() {
		return (leftMotorControllers[0].getClosedLoopTarget(0)-leftMotorControllers[0].getSelectedSensorPosition(0))<40
				&& (rightMotorControllers[0].getClosedLoopTarget(0)-rightMotorControllers[0].getSelectedSensorPosition(0))<40;
	}
	
	public void periodicTasks() {
		//publish left and right encoder Position to Dashboard.
		SmartDashboard.putString("DB/String 0", "Pl:" + leftMotorControllers[0].getSelectedSensorPosition(0));
		SmartDashboard.putString("DB/String 1", "Pr:" + rightMotorControllers[0].getSelectedSensorPosition(0));
		SmartDashboard.putString("DB/String 2", "Vl:" + leftMotorControllers[0].getSelectedSensorVelocity(0));
		SmartDashboard.putString("DB/String 3", "Vr:" + rightMotorControllers[0].getSelectedSensorVelocity(0));
		SmartDashboard.putBoolean("DB/LED 0", isPtoEngaged());
		SmartDashboard.putString("DB/String 4", "H" + getHeading());
		
		if(isPathFollowing == true) {
			leftMotorControllers[0].getMotionProfileStatus(leftStat);
			rightMotorControllers[0].getMotionProfileStatus(rightStat);
			//System.out.println("T-points remaining left  : top: " + leftStat.topBufferCnt + " bottom: " + leftStat.btmBufferCnt);
			//System.out.println("T-points remaining right : top: " + rightStat.topBufferCnt + " bottom: " + rightStat.btmBufferCnt);
			if(leftStat.hasUnderrun || rightStat.hasUnderrun) {
				System.out.println("Has Underrun: left:" + leftStat.hasUnderrun + " right:" + rightStat.hasUnderrun);
				leftMotorControllers[0].clearMotionProfileHasUnderrun(0);
				rightMotorControllers[0].clearMotionProfileHasUnderrun(0);
			}
			if(leftStat.isLast && rightStat.isLast) {
				isPathFollowing = false;
				leftMotorControllers[0].set(ControlMode.MotionProfile,SetValueMotionProfile.Hold.value);
				rightMotorControllers[0].set(ControlMode.MotionProfile,SetValueMotionProfile.Hold.value);
				if(isLogging) {
					stopLogging();
				}
			}
		}
	}
	

    public void initDefaultCommand() {
       setDefaultCommand(new DriveDefault());
    }
    
    public void processMotionProfileBuffer() {
    		leftMotorControllers[0].processMotionProfileBuffer();
    		rightMotorControllers[0].processMotionProfileBuffer();
    }
    
    public void configOpenLoopRampRate(double seconds) {
    		leftMotorControllers[0].configOpenloopRamp(seconds, 0);
    		rightMotorControllers[0].configOpenloopRamp(seconds, 0);
    }
    
    public void configMaxOutputVoltage(double volts) {
    		leftMotorControllers[0].configVoltageCompSaturation(volts, 0);
    		rightMotorControllers[0].configVoltageCompSaturation(volts, 0);
    }
}

