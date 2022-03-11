// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Base.AimWithLimelight;
import frc.robot.commands.Base.DriveToPose;
import frc.robot.commands.Base.ResetGyro;
import frc.robot.commands.Base.ResetOdometry;
import frc.robot.commands.Flywheel.FlywheelSpinWithLimelight;
import frc.robot.commands.Miscellaneous.DeadlineTimer;
import frc.robot.commands.Storage.StorageSpinIntoFlywheel;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.NeoBase;
import frc.robot.subsystems.Storage;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuton extends SequentialCommandGroup {
  /** Creates a new TestAuton. */
  public TestAuton(NeoBase base, Camera camera, Flywheel flywheel, Storage storage) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetOdometry(base),
      new ResetGyro(base),

      new DriveToPose(base, new Pose2d(-2, 0, new Rotation2d())),

      new ParallelDeadlineGroup(new WaitCommand(2.5),
        new AimWithLimelight(base, camera),
        new FlywheelSpinWithLimelight(flywheel, camera)
      ),

      new ParallelDeadlineGroup(new WaitCommand(6), 
        new AimWithLimelight(base, camera),
        new FlywheelSpinWithLimelight(flywheel, camera),
        new StorageSpinIntoFlywheel(storage)
      )
    );
  }
}
