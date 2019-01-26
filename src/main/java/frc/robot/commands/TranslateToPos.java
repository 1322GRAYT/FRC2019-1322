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

    int X = 0, Y = 0;
    int[] motorDistance = new int[4];

    public TranslateToPos(int x, int y) {
        X = x;
        Y = y;
        requires(Robot.DRIVES);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.DRIVES.RelativeMotionMagic(X, Y);

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

}