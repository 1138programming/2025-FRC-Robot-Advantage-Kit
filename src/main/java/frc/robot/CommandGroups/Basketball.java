// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CommandGroups.LiftSetpoints.LiftandArmTier4;
import frc.robot.commands.DriveStop;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.drive.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Basketball extends SequentialCommandGroup {
  /** Creates a new Basketball. */
  Drive drive;

  public Basketball(Drive drive, Arm arm, Lift lift) {
    // Add your commands in

    addCommands(new DriveForwardTime(drive, 3.0, 0.2));
    addCommands(new DriveStop(drive));
    addCommands(new LiftandArmTier4(arm, lift));
  }
}
