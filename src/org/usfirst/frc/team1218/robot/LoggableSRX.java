package org.usfirst.frc.team1218.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class LoggableSRX extends TalonSRX {
	private String deviceName;
	
	private class LoggerThread extends Thread {

		@Override
		public void run() {
			// TODO Auto-generated method stub
			super.run();
		}

		@Override
		public synchronized void start() {
			// TODO Auto-generated method stub
			super.start();
		}
		
	}
	
	
	public LoggableSRX(int deviceNumber, String dn) {
		super(deviceNumber);
		deviceName = dn;
		this.
	}

	
}
