/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;

/** Class: CA_TmDly - Delays the Robot for the time value supplied
 *         as an argument by the calling Command Group (seconds).  
 * @param: Desired Delay Time (seconds)
 */
public class CA_TmDly extends Command {
    private Float DlyTmThrsh;
	
	  private Timer timer = new Timer();

    public CA_TmDly(Float DlyTmThrsh) {
    	this.DlyTmThrsh = DlyTmThrsh;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
//   obot.AUTON.setMasterTaskCmplt(false);
    timer.reset();
    timer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Just Wait - Timer is Free Running.
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (timer.get() >= (double)DlyTmThrsh);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    timer.stop();
//  Robot.kAUTON.setMasterTaskCmplt(true);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
