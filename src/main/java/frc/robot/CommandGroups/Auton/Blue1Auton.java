// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Base.DriveToPose;
import frc.robot.commands.Flywheel.FlywheelSpin;
import frc.robot.commands.Flywheel.FlywheelSpinWithLimelight;
import frc.robot.commands.Intake.HuntMode;
import frc.robot.commands.Miscellaneous.DeadlineTimer;
import frc.robot.commands.Storage.BottomStorageIn;
import frc.robot.commands.Storage.StorageCollect;
import frc.robot.commands.Storage.TopStorageIn;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NeoBase;
import frc.robot.subsystems.Storage;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Blue1Auton extends SequentialCommandGroup {
  /** Creates a new Blue1Auton. */
  public Blue1Auton(NeoBase base, Flywheel flywheel, Hang hang, Intake intake, Storage storage, Camera camera) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(new DeadlineTimer(1000),
        new FlywheelSpinWithLimelight(flywheel, camera)
      ),
      
      new ParallelDeadlineGroup(new DeadlineTimer(2000), 
        new FlywheelSpinWithLimelight(flywheel, camera),
        new TopStorageIn(storage), 
        new BottomStorageIn(storage)
      ),

      new ParallelRaceGroup(
        new DriveToPose(base, new Pose2d(1, 0, new Rotation2d())),
        new HuntMode(intake),
        new StorageCollect(storage)
      ),
      
      new DriveToPose(base, new Pose2d(0, 0, new Rotation2d())),
      
      new ParallelDeadlineGroup(new DeadlineTimer(2000), 
        new FlywheelSpinWithLimelight(flywheel, camera),
        new TopStorageIn(storage), 
        new BottomStorageIn(storage)
      ),

      new ParallelRaceGroup(
        new DriveToPose(base, new Pose2d(1, 0, new Rotation2d())),
        new HuntMode(intake),
        new StorageCollect(storage)
      ),

      new DriveToPose(base, new Pose2d(0, 0, new Rotation2d())),
      
      new ParallelDeadlineGroup(new DeadlineTimer(2000), 
        new FlywheelSpinWithLimelight(flywheel, camera),
        new TopStorageIn(storage), 
        new BottomStorageIn(storage)
      )
    );
  }
}
