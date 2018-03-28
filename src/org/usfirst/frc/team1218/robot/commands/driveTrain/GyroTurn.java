package org.usfirst.frc.team1218.robot.commands.driveTrain;

import org.usfirst.frc.team1218.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;

/**
 *
 */
public class GyroTurn extends Command implements PIDOutput{
	
	double[] GryoPIDF = {0.0,0.0,0.0,0.0};
	PIDController controller;
	double setpoint;
	
    public GyroTurn(double setpoint) {
        requires(Robot.driveTrain);
        this.setpoint = setpoint;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	controller = new PIDController(GryoPIDF[0], GryoPIDF[1], GryoPIDF[2], GryoPIDF[3], Robot.driveTrain.navx, this);
    	controller.setOutputRange(-0.7, 0.7);
    	controller.setInputRange(-180.0, 180.0);
    	controller.setContinuous(true);
    	controller.setSetpoint(setpoint);
    	controller.setPercentTolerance(1.0);
    	controller.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return controller.onTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

	@Override
	public void pidWrite(double output) {
		Robot.driveTrain.setPower(-output, output);
		
	}
}
