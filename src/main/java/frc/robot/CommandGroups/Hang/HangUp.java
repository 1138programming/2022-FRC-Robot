// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Hang;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Hang.MoveHangUp;
import frc.robot.commands.Hang.MoveRachetIn;
import frc.robot.commands.Hang.MoveHangDown;
import frc.robot.commands.Hang.MoveHangDownFor;
import frc.robot.commands.Hang.MoveRachetOut;
import frc.robot.commands.Miscellaneous.DeadlineTimer;
import frc.robot.subsystems.Hang;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HangUp extends SequentialCommandGroup {
  /** Creates a new hangUp. */
  public HangUp(Hang hang) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveRachetIn(hang),
      new ParallelRaceGroup(
        new WaitCommand(0.05),
        new MoveHangDown(hang)
        // new MoveRachetIn(hang)
      ),
      new MoveHangUp(hang)
    );
  }
}
