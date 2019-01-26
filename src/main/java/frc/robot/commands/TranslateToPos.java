/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Arrays;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TranslateToPos extends Command {

    int[] motorDistance = new int[4];
    int pHolder = 0;

    public TranslateToPos(int x, int y) {
        pHolder = (x != 0 ? x : y);
        requires(Robot.DRIVES);
        final int[] motorD = { (int) Math.signum(x) * pHolder, (int) Math.signum(-x) * pHolder,
            (int) Math.signum(-x) * pHolder, (int) Math.signum(x) * pHolder }; // Determine which direction the robot needs to go
        motorDistance = motorD;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        Robot.DRIVES.RelativeMotionMagic(X, Y);
    }

    // Make this return true when this Command no longer needs to run execute()
    final boolean[] flagCheck = { true, true, true, true };

    @Override
    protected boolean isFinished() {
        return Arrays.equals(Robot.DRIVES.mmIsDone(), flagCheck);
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.DRIVES.DriveInVoltage(0, 0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }

private int[] relativePosition = new int[4];
  public void setRelativePosition() {
      relativePosition = Robot.DRIVES.rawiPosition();
  }

}