package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TC_LiftMotor extends Command {

    public TC_LiftMotor() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.LIFT);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//When Robot starts, disengage the jammer
    	Robot.LIFT.disengageJammer(); //TODO:  SEE IF THIS IS CAUSING ISSUES
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//Set speed of lift motors
    	Robot.LIFT.setSpeed(-Robot.m_oi.gamePad2.getY(Hand.kRight));
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}