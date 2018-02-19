package org.usfirst.frc.team1218.robot.subsystems;

import java.util.concurrent.atomic.AtomicInteger;

import org.team1218.lib.ctrlSystemLogging.LoggableSRX;
import org.usfirst.frc.team1218.robot.RobotMap;
import org.usfirst.frc.team1218.robot.commands.elevator.ElevatorDefault;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Elevator extends Subsystem {
	
	//static final boolean invertElevator = false;
	//static final boolean[] invertIntake = {true,false};

	boolean intakeStatus = false;
	boolean isLogging = false;
	//static final double[] elevatorGains = {0.0, 0.0, 0.0, 0.0};  //PIDF for Motion Magic
	//static final int elevatorCruiseVelocity = 0;		// encoder ticks / 100ms
	//static final int elevatorAcceleration = 0;			// change in verlocity per sec, i.e. when accel = cruise, will take 1sec to reach cruise
	LoggableSRX[] elevatorMotors = new LoggableSRX[2];
	TalonSRX[] intakeMotors = new TalonSRX[2];
	Solenoid intakeSolenoid;
	Solenoid armSolenoid;
	Faults elevatorFaults = new Faults();
	private AtomicInteger motionMagicPos = new AtomicInteger(0);
	
	private Notifier processMPBuffer = new Notifier(new Runnable() {

		@Override
		public void run() {
			elevatorMotors[0].processMotionProfileBuffer();
			
		}
		
	});
	
	public Elevator() {
		for(int i = 0; i < 2; i ++) {
			elevatorMotors[i] = new LoggableSRX(RobotMap.elevatorMotorIds[i]);
			elevatorMotors[i].setInverted(RobotMap.elevatorMotorInvert);
			elevatorMotors[i].enableVoltageCompensation(true);
			
			intakeMotors[i] = new TalonSRX(RobotMap.intakeMotorIds[i]);
			intakeMotors[i].setInverted(RobotMap.intakeMotorInvert[i]);
			intakeMotors[i].enableVoltageCompensation(true);
		}
		
		
		for(int i = 1; i < 2; i++) {
			elevatorMotors[i].set(ControlMode.Follower, RobotMap.elevatorMotorIds[0]);
			intakeMotors[i].set(ControlMode.Follower, RobotMap.intakeMotorIds[0]);
		}
		
		elevatorMotors[0].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		elevatorMotors[0].setSensorPhase(false);
		elevatorMotors[0].configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
		elevatorMotors[0].configForwardSoftLimitThreshold(275000, 0);
		elevatorMotors[0].configForwardSoftLimitEnable(true, 0);
		
		elevatorMotors[0].config_kP(0, RobotMap.elevatorPIDF[0], 0);
		elevatorMotors[0].config_kI(0, RobotMap.elevatorPIDF[1], 0);
		elevatorMotors[0].config_kD(0, RobotMap.elevatorPIDF[2], 0);
		elevatorMotors[0].config_kF(0, RobotMap.elevatorPIDF[3], 0);
		
		elevatorMotors[0].setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10, 0);
		elevatorMotors[0].setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10, 0);
		
		elevatorMotors[0].configMotionCruiseVelocity(RobotMap.elevatorCruiseVelocity, 0);
		elevatorMotors[0].configMotionAcceleration(RobotMap.elevatorAcceleration, 0);
		
		intakeSolenoid = new Solenoid(RobotMap.intakePort);
		armSolenoid = new Solenoid(RobotMap.armPort);
		System.out.println("Elevator Kf: " + RobotMap.elevatorPIDF[3]);
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
	
	public void executeMotionMagicMove() {
		System.out.println("Elevator.executeMotionMagicMove to pos: " + motionMagicPos);
		elevatorMotors[0].set(ControlMode.MotionMagic, motionMagicPos.get());
	}

	public void setMotionMagicPosition(int newPos) {
		motionMagicPos.set(newPos);
	}
	
	public int getMotionMagicPosition() {
		return motionMagicPos.get();
	}
	
	public void moveTo(int position) {
		System.out.println("moving to " + position);
		elevatorMotors[0].set(ControlMode.MotionMagic, position);
	}
	
	public void setIntakePower(double intakePower) {
		intakeMotors[0].set(ControlMode.PercentOutput, intakePower);
	}
	
	public void intakeSolenoidEngage(boolean intakeState) {
		intakeSolenoid.set(intakeState);
	}
	public void armSolenoidEngage(boolean armState) {
		armSolenoid.set(!armState);
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
		setDefaultCommand(new ElevatorDefault());
		
	}

}
