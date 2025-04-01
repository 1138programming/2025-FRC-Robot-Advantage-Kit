// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;
import frc.robot.util.SubsystemUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralDefault extends Command {
  private final CoralIntake coralIntake;
  private SubsystemUtil util;
  /** Creates a new SpinCoralIntake. */
  public CoralDefault(CoralIntake coralIntake, SubsystemUtil util) {
    this.coralIntake = coralIntake;
    this.util = util;

    addRequirements(coralIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (util.getArmSpeed() > 10 || util.getArmSpeed() < -10) {
      coralIntake.setCoralIntakeSpeed(0.7);
    } else {
      coralIntake.setCoralIntakeSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
