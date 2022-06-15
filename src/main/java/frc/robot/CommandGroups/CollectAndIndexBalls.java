// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import frc.robot.commands.Intake.HuntMode;
import frc.robot.commands.Intake.IntakeSpinForward;
import frc.robot.commands.Storage.StorageCollect;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Storage;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CollectAndIndexBalls extends SequentialCommandGroup {
  /* Creates a new CollectAndIndexBalls. */
  public CollectAndIndexBalls(Intake intake, Storage storage) {
    addCommands(
      new ParallelCommandGroup(
        new ParallelCommandGroup(
            new HuntMode(intake),
            new StorageCollect(storage)
        )
      )
    );
  }
}
