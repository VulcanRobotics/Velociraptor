package org.team1218.lib.ctrlSystemLogging;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicBoolean;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import mjson.Json;

public class LoggableSRX extends TalonSRX {
	protected String deviceName;
	protected BufferedWriter log;
	protected Timer loggerTimer;
	protected AtomicBoolean endFlag = new AtomicBoolean(false);;
	
	private class LoggerTask extends TimerTask {
		long startTime, time, dt;
		Json data;
		LoggableSRX srx;
		int count = 0;
		double vel, pos, accel;
		
		public LoggerTask(LoggableSRX srx) {
			this.srx = srx;
			data = Json.object();
			data.set("timeStamp", Json.array());
			data.set("error", Json.array());
			data.set("setpoint", Json.array());
			data.set("velocity", Json.array());
			data.set("position", Json.array());
			data.set("vOut", Json.array());
			data.set("acceleration", Json.array());
			data.set("dt", Json.array());
			startTime = System.currentTimeMillis();
		}
		
		@Override
		public void run() {			
			time = System.currentTimeMillis() - startTime;
			data.at("timeStamp").add(time);
			if(srx.getControlMode() == ControlMode.Velocity || srx.getControlMode() == ControlMode.MotionMagic) {
				data.at("error").add(srx.getClosedLoopError(0));
				data.at("setpoint").add(srx.getClosedLoopTarget(0));
			} else if(srx.getControlMode() == ControlMode.MotionProfile){
				int pos = srx.getActiveTrajectoryPosition();
				data.at("setpoint").add(pos);
				data.at("error").add(srx.getSelectedSensorPosition(0) - pos);
			}else {
				data.at("error").add(0);
				data.at("setpoint").add(0);
			}
			
			vel = srx.getSelectedSensorVelocity(0);
			pos = srx.getSelectedSensorPosition(0);
			data.at("velocity").add(vel);
			data.at("position").add(pos);
			data.at("vOut").add(srx.getMotorOutputVoltage());
			if (++count >= 3) {
				dt = time - data.at("timeStamp").at(count - 3).asLong();
				accel = (vel - data.at("velocity").at(count - 3).asDouble()) / dt;
				data.at("acceleration").add(accel);
				data.at("dt").add(dt);
			} else {
				data.at("acceleration").add(0.0);
				data.at("dt").add(0);
			}
			if(endFlag.get() == true) {
				endFlag.set(false);
				try {
					System.out.println("Writting Log to File");;
					srx.log.write(data.toString());
					srx.log.close();
					
				} catch (IOException e) {
					System.out.println("IO Exception in" + deviceName +": Stop Logging" );
					e.printStackTrace();
				}finally {
					System.out.println("ending log on " + deviceName);
					srx.loggerTimer.cancel();
				}
			}
		}
	}
	
	
	public LoggableSRX(int deviceNumber, String deviceName) {
		super(deviceNumber);
		this.deviceName = deviceName;
	}
	
	public LoggableSRX(int deviceNumber) {
		this(deviceNumber, "TalonSRX" + deviceNumber);
	}
	
	public boolean startLogging() {
		try {
			loggerTimer = new Timer();
			File logfile = new File("/home/lvuser/log/talonSRX/" + deviceName + ".json");
			logfile.createNewFile();
			log = new BufferedWriter(new FileWriter(logfile));
			endFlag.set(false);
			loggerTimer.scheduleAtFixedRate(new LoggerTask(this), 0, 20);
			System.out.println("Starting Logging on " + deviceName);
			return true;
		} catch (IOException e) {
			System.out.println("IO Exception in" + deviceName +": Stop Logging" );
			e.printStackTrace();
			return false;
		}
	}
	
	public void stopLogging() {
		System.out.println("set stop flag");
		endFlag.set(true);
	}

	
}
