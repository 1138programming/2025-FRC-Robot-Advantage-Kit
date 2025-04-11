// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.AutoDrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CommandGroups.LiftSetpoints.LiftandArmTier4;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Lift;
import java.io.IOException;
import org.json.simple.parser.ParseException;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScore extends SequentialCommandGroup {
  private String Name;
  private Lift lift;
  private Arm arm;

  /**
   * Creates a new AutoScore.
   *
   * @throws ParseException
   * @throws IOException
   * @throws FileVersionException
   */
  public AutoScore(String Name, Lift lift, Arm arm) {
    this.Name = Name;
    this.arm = arm;
    this.lift = lift;

    PathPlannerPath path;
    try {
      path = PathPlannerPath.fromPathFile(Name);
    } catch (FileVersionException e) {
      // TODO Auto-generated catch block
      path = null;
      e.printStackTrace();
    } catch (IOException e) {
      path = null;
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (ParseException e) {
      // TODO Auto-generated catch block
      path = null;
      e.printStackTrace();
    }

    PathConstraints constraints =
        new PathConstraints(4.65, 3, Units.degreesToRadians(540), Units.degreesToRadians(720));
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
    addCommands(
        pathfindingCommand,
        new ParallelDeadlineGroup(new WaitCommand(0.3), new LiftandArmTier4(arm, lift)));
  }
}
