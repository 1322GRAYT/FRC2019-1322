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

public class TranslateToPos extends Command {

    int[] motorDistance = new int[4];
    int pHolder = 0;
    int EError = 0; // For Error
    final static int TOLERANCE = 250;

    public TranslateToPos(int x, int y) {
        requires(Robot.DRIVES);

        /********************
         * TODO: In the near future, I will need to be able to ensure that the PID slot
         * I choose to use gets set, as I will be setting the gyro to be using another
         * slot 1/27/19
         */

        // Only moving one at once, but must decern which way to go first
        // In this game, side to side motion first is the most important
        if (x != 0) {
            final int[] motorD = { (int) Math.signum(x) * x, (int) Math.signum(-x) * x, (int) Math.signum(-x) * x,
                    (int) Math.signum(x) * x };
            motorDistance = motorD;
        } else {
            final int[] motorD = { y, y, y, y };
            motorDistance = motorD;
        }

        // We are trying not to reset the encoders, so we need to move with respect to
        // where
        // we are currently
        setRelativePosition();
        for (int i = 0; i < motorDistance.length; i++) {
            motorDistance[i] += relativePosition[i];
        }

        EError = (Robot.DRIVES.rawiPosition()[0] - pHolder);

    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        System.out.println("Initializing");
        Robot.DRIVES.setSafety(false);
        Robot.DRIVES.MMControlTest(this.motorDistance[0]);
    }

    // Called repeatedly when this Command is scheduled to run
    // Creating output Vals
    double PosE = 0;
    double Vel = 0;
    double CLE = 0;

    @Override
    protected void execute() {
        PosE = Robot.DRIVES.getClosedLoopError()[0];
        Vel = Robot.DRIVES.rawVelocities()[0];
        CLE = Robot.DRIVES.getClosedLoopError()[0];
        EError = (int) (Robot.DRIVES.rawPosition()[0] - pHolder);
    }

    // Make this return true when this Command no longer needs to run execute()

    @Override
    protected boolean isFinished() {
        return Math.abs(EError) < TOLERANCE;
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

    private int[] relativePosition = new int[4];

    public void setRelativePosition() {
        relativePosition = Robot.DRIVES.rawiPosition();
    }

    public void toSDBoard(double Error, double Velocity, double CLE, double SetPoint) {
        double[] toSDB = { Error, Velocity, CLE, SetPoint };
        SmartDashboard.putNumberArray("Translation Data", toSDB);
    }

}