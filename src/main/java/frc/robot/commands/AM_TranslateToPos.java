/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class AM_TranslateToPos extends Command {

    int[] motorDistance = new int[4];
    int pHolder = 0;
    int EError = TOLERANCE + 1; // For Error
    int x = 0, y = 0; // Placeholders
    final static int TOLERANCE = 500;

    public AM_TranslateToPos(int x, int y) {
        requires(Robot.DRIVES);
        this.x = x;
        this.y = y;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {

        // First we need to create a setpoint, we need to determine which value we are
        // taking
        if (x != 0) {
            final int[] motorD = { (int) Math.signum(x) * x, (int) Math.signum(-x) * x, (int) Math.signum(-x) * x,
                    (int) Math.signum(x) * x };
            motorDistance = motorD;
        } else {
            final int[] motorD = { y, y, y, y };
            motorDistance = motorD;
        }

        // We are trying not to reset the encoders, so we need to move with respect to
        // where we are currently
        for (int i = 0; i < motorDistance.length; i++) {
            motorDistance[i] += Robot.DRIVES.rawPosition()[i];
        }
        
        Robot.DRIVES.setSafety(false);
        Robot.DRIVES.MMControl(motorDistance);
    }

    // Called repeatedly when this Command is scheduled to run

    @Override
    protected void execute() {
        // Display Command Stats
        toSDBoard("Drive 1", calcError(0), Robot.DRIVES.rawVelocities()[0],
                Robot.DRIVES.getClosedLoopError()[0], motorDistance[0]);
        toSDBoard("Drive 2", calcError(1), Robot.DRIVES.rawVelocities()[1],
                Robot.DRIVES.getClosedLoopError()[1], motorDistance[1]);
        toSDBoard("Drive 3", calcError(2), Robot.DRIVES.rawVelocities()[2],
                Robot.DRIVES.getClosedLoopError()[2], motorDistance[2]);
        toSDBoard("Drive 4", calcError(3), Robot.DRIVES.rawVelocities()[3],
                Robot.DRIVES.getClosedLoopError()[3], motorDistance[3]);
    }

    // Make this return true when this Command no longer needs to run execute()

    @Override
    protected boolean isFinished() {
        return Math.abs(calcError(0)) < TOLERANCE;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.DRIVES.DriveInVoltage(0, 0, 0);
        Robot.DRIVES.setSafety(true);
        System.out.println("Done");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
        System.out.println("Interrupted");
    }


    public int calcError(int motor){
        return (int)Robot.DRIVES.rawPosition()[motor] - motorDistance[motor];
    }

    public void toSDBoard(String Name,double... toSDB) {
        SmartDashboard.putNumberArray(Name, toSDB);
    }

}