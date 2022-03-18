// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CommandGroups.AutonFeedShot;
import frc.robot.CommandGroups.FeedShot;
import frc.robot.commands.Base.AimWithLimelight;
import frc.robot.commands.Base.DriveToPose;
import frc.robot.commands.Base.ResetGyro;
import frc.robot.commands.Base.ResetOdometry;
import frc.robot.commands.Base.RotateToHeading;
import frc.robot.commands.Flywheel.FlywheelSpinWithLimelight;
import frc.robot.commands.Intake.IntakeSpinForward;
import frc.robot.commands.Intake.IntakeSwivelDown;
import frc.robot.commands.Storage.StorageCollect;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NeoBase;
import frc.robot.subsystems.Storage;

// NOTE: Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveBackAndShoot extends SequentialCommandGroup {
  public DriveBackAndShoot(NeoBase base, Camera camera, Storage storage, Intake intake, Flywheel flywheel) {
    addCommands(
      // new ParallelRaceGroup(new WaitCommand(0.8),
      //   new IntakeSwivelDown(intake)
      // ),
      // new ParallelRaceGroup(new WaitCommand(0.8),
      //   new IntakeSpinForward(intake),
      //   new StorageCollect(storage)
      // ),
      
      
      new ParallelRaceGroup(new WaitCommand(0.3),
        new IntakeSwivelDown(intake)
      ),
      new ParallelRaceGroup(
          new WaitCommand(6),
          new FlywheelSpinWithLimelight(flywheel, camera),
          new AutonFeedShot(storage)
        ),
      new ResetOdometry(base),
      new ResetGyro(base),
      new DriveToPose(base, new Pose2d(-1.5, 0, new Rotation2d()))


      // new AimWithLimelight(base, camera),
    );
  }
}
