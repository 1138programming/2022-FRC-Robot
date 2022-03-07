// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Hang;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Hang.MoveArmsToPosition;
import frc.robot.commands.Hang.MoveClawOut;
import frc.robot.commands.Hang.MoveLevelHangTo;
import frc.robot.subsystems.Hang;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HangToTraversal extends SequentialCommandGroup {
  /** Creates a new HangToTraversal. */
  public HangToTraversal(Hang hang) {
    addCommands(
      new MoveArmsToPosition(hang, -30),
      new MoveLevelHangTo(hang, 0),
      new MoveClawOut(hang),
      new HangToNextBar(hang),
      new HangToNextBar(hang),
      new HangToNextBar(hang)
    );
  }
}
