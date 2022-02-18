// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Hang;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.Hang.MoveArms;
import frc.robot.commands.Hang.MoveArmsToLimit;
import frc.robot.commands.Hang.MoveLift;
import frc.robot.commands.Hang.MoveLiftToBottomLimit;
import frc.robot.commands.Hang.MoveLiftToTopLimit;
import frc.robot.commands.Hang.MoveClaw;
import frc.robot.subsystems.Hang;
import static frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HangToNextBar extends SequentialCommandGroup {
  /** Creates a new HangToNextBar. */
  public HangToNextBar(Hang hang) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveLiftToTopLimit(hang),
      new MoveArmsToLimit(hang),
      new MoveClaw(hang),
      new MoveLiftToBottomLimit(hang)
    );
  }
}
