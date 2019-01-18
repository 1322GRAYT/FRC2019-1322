/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class TransToPos extends CommandGroup {

  public TransToPos(double x, double y) {
    if (x != 0){
      addSequential(new TransToPos(x, 0));
    }

    if (y != 0){
      addSequential(new TransToPos(0, y));
    }
  }
}
