/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1218.robot;

import org.usfirst.frc.team1218.robot.commands.driveTrain.CharacterizeDriveTrain;
import org.usfirst.frc.team1218.robot.commands.driveTrain.ToggleDriveTrainLogging;
import org.usfirst.frc.team1218.robot.commands.driveTrain.TogglePTO;
import org.usfirst.frc.team1218.robot.commands.elevator.ElevatorTest;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI{
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
	
	public Joystick driver;
	public Joystick operator;
	public Button followPathBtn; 
	public Button shiftBtn;
	public Button loggingBtn;
	public Button ptoBtn;
	public Button intakeBtn;
	public Button intakeArmBtn;
	public Button outtakeBtn;
	public Button armUpBtn;
	public Button armDownBtn;
	public Button elevatorCommandTestBtn;
	
	public OI() {
		driver = new Joystick(0);
		operator = new Joystick(1);
		followPathBtn = new JoystickButton(driver, 2);
		shiftBtn = new JoystickButton(driver,1);
		loggingBtn = new JoystickButton(driver,3);
		loggingBtn.whenActive(new ToggleDriveTrainLogging());
		ptoBtn = new JoystickButton(driver,4);
		ptoBtn.whenActive(new TogglePTO());
		intakeBtn = new JoystickButton(operator,2);
		intakeArmBtn = new JoystickButton(operator,1);
		outtakeBtn = new JoystickButton(operator,3);
		armUpBtn = new JoystickButton(operator,6);
		armDownBtn = new JoystickButton(operator,4);
		new JoystickButton(operator, 9).whileHeld(new CharacterizeDriveTrain());
		elevatorCommandTestBtn = new JoystickButton(operator,11);
		elevatorCommandTestBtn.whenPressed(new ElevatorTest());
		
	}
	
}
