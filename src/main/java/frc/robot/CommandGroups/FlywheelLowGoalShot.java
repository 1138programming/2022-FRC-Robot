// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Flywheel.FlywheelSpinAtRPM;
import frc.robot.commands.Storage.BottomStorageIn;
import frc.robot.commands.Storage.StorageSpinIntoFlywheel;
import frc.robot.commands.Storage.TopStorageIn;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Storage;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FlywheelLowGoalShot extends SequentialCommandGroup {
  /** Creates a new FlywheelLowGoalShot. */
  public FlywheelLowGoalShot(Flywheel flywheel, Storage storage) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new FlywheelSpinAtRPM(flywheel, 1150),
        new StorageSpinIntoFlywheel(storage)
      )     
    );  
  }
}
