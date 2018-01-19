package org.usfirst.frc.team1218.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicBoolean;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import mjson.Json;

public class LoggableSRX extends TalonSRX {
	protected String deviceName;
	protected BufferedWriter log;
	protected Timer loggerTimer;
	protected AtomicBoolean endFlag = new AtomicBoolean(false);;
	
	private class LoggerTask extends TimerTask {
		long startTime;
		Json data;
		LoggableSRX srx;
		public LoggerTask(LoggableSRX srx) {
			this.srx = srx;
			data = Json.object();
			data.set("timeStamp", Json.array());
			data.set("error", Json.array());
			data.set("setpoint", Json.array());
			data.set("velocity", Json.array());
			data.set("position", Json.array());
			startTime = System.currentTimeMillis();
		}
		@Override
		public void run() {
			data.at("timeStamp").add(System.currentTimeMillis() - startTime);
			data.at("error").add(srx.getClosedLoopError(0));
			data.at("setpoint").add(srx.getClosedLoopTarget(0));
			data.at("velocity").add(srx.getSelectedSensorVelocity(0));
			data.at("position").add(srx.getSelectedSensorPosition(0));
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
			log.write("Error,Setpoint,Velocity\n");
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
