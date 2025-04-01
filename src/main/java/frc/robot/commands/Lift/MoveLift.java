package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift;

public class MoveLift extends Command {
  private Lift lift;
  private double speed;

  public MoveLift(Lift lift, double speed) {
    this.lift = lift;
    this.speed = speed;
    addRequirements(lift);
  }

  @Override
  public void execute() {
    lift.setLiftElevatorSpeed(speed);
  }

  @Override
  public void end(boolean interrDownted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
