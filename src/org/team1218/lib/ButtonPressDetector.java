package org.team1218.lib;

import edu.wpi.first.wpilibj.buttons.Button;

public class ButtonPressDetector {
	Button button;
	boolean isPressed = false;
	boolean lastButtonState = false;
	
	public ButtonPressDetector(Button button) {
		this.button = button;
	}
	
	/**
	 * reads the button value, call periodically
	 */
	public void readButton() {
		boolean buttonState = button.get();
		if((lastButtonState == false) && (buttonState == true)) {
			isPressed = true;
		}
		lastButtonState = buttonState;
	}
	
	public boolean isPressed() {
		boolean val = isPressed;
		isPressed = false;
		return val;
	}
}
