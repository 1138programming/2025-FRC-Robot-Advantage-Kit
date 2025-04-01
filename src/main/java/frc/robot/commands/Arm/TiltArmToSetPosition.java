package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class TiltArmToSetPosition extends Command {

  private Arm arm;
  private double position;

  // command requires an arm and a position to tilt to
  public TiltArmToSetPosition(Arm arm, double position) {
    this.arm = arm;
    this.position = position;
    addRequirements(arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    arm.tiltArmToSetPositionWPI(position);
  }

  @Override
  public void end(boolean interrupted) {
    // if (interrupted) {
    arm.stop();
    // }
  }

  @Override
  public boolean isFinished() {
    // return arm.isAtSetPosition();
    return false;
  }
}
