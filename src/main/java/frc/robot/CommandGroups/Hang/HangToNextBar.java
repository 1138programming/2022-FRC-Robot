// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Hang;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Base.DriveToPose;
import frc.robot.commands.Base.ResetOdometry;
import frc.robot.commands.Hang.MoveArmBackward;
import frc.robot.commands.Hang.MoveArmForward;
import frc.robot.commands.Hang.MoveArmsToPosition;
import frc.robot.commands.Hang.MoveClawIn;
import frc.robot.commands.Hang.MoveClawOut;
import frc.robot.commands.Hang.MoveHangToLimit;
import frc.robot.commands.Hang.MoveLevelHangTo;
import frc.robot.commands.Miscellaneous.DeadlineTimer;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.NeoBase;

import static frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HangToNextBar extends SequentialCommandGroup {
  /** Creates a new HangToNextBar. */
  public HangToNextBar(Hang hang, NeoBase base) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveArmsToPosition(hang, -30),  // ALL VALUES ARE RANDOM AND WON'T WORK AND THIS WILL ONLY CLIMB TO HIGH
      new MoveLevelHangTo(hang, 100),
      new ResetOdometry(base),
      new ParallelDeadlineGroup(new DeadlineTimer(500), 
        new DriveToPose(base, new Pose2d(0.5, 0, new Rotation2d()))
      ),
      new MoveArmsToPosition(hang, 0),
      new MoveClawOut(hang),
      new MoveLevelHangTo(hang, 50),
      new MoveArmsToPosition(hang, 45),
      new MoveHangToLimit(hang),
      new MoveArmsToPosition(hang, 0),
      new MoveLevelHangTo(hang, 0),
      new MoveClawIn(hang),
      new MoveArmsToPosition(hang, -30)
    );
  }
}
