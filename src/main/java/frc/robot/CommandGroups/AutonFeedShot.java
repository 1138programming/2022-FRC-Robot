// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Flywheel.FlywheelSpinWithLimelight;
import frc.robot.commands.Intake.IntakeSpinForward;
import frc.robot.commands.Storage.StorageCollect;
import frc.robot.commands.Storage.StorageSpinIntoFlywheel;
import frc.robot.subsystems.Storage;
import static frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonFeedShot extends SequentialCommandGroup {
  /** Creates a new FeedShot. */
  public AutonFeedShot(Storage storage) {

    addCommands(
      new ParallelRaceGroup(
        new WaitCommand(0.3),
        new StorageSpinIntoFlywheel(storage)
      ),

      new ParallelDeadlineGroup(
        new WaitCommand(0.2),
        new StorageCollect(storage)
      ),

      new ParallelRaceGroup(
        new WaitCommand(0.35),
        new StorageSpinIntoFlywheel(storage, KTopStorageAutonPWM)
      )      
    );
  }
}
