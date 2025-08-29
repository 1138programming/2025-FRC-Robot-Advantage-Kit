// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveSidetoSideTime extends ParallelDeadlineGroup {
  Drive drive;
  Double speed2;
  /** Creates a new DriveSidetoSideTime. */
  public DriveSidetoSideTime(Drive drive, Double seconds, Double speed) {
    super(new WaitCommand(seconds));
    this.drive = drive;

    speed2 = Math.min(speed, 0.2);
    speed2 = Math.max(speed, -0.2);
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(DriveCommands.Drive(drive, () -> 0, () -> speed2, () -> 0));
  }
}
