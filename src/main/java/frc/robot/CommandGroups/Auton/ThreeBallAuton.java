// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CommandGroups.AutonFeedShot;
import frc.robot.commands.Base.AimWithLimelight;
import frc.robot.commands.Base.DriveToPose;
import frc.robot.commands.Base.ResetGyro;
import frc.robot.commands.Base.ResetOdometry;
import frc.robot.commands.Base.RotateToHeading;
import frc.robot.commands.Flywheel.FlywheelSpinWithLimelight;
import frc.robot.commands.Intake.HuntMode;
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
public class ThreeBallAuton extends SequentialCommandGroup {
  public ThreeBallAuton(NeoBase base, Camera camera, Storage storage, Intake intake, Flywheel flywheel) {
    addCommands(
      new ResetGyro(base, 90),
      new ResetOdometry(base),
      new ParallelRaceGroup(
        new WaitCommand(0.3),
        new IntakeSwivelDown(intake)
      ),
      new ParallelRaceGroup(new WaitCommand(6),
        new FlywheelSpinWithLimelight(flywheel, camera),
        new AutonFeedShot(storage)
      ),
      new ParallelRaceGroup(new WaitCommand(2),
        new RotateToHeading(base, -90)
      ),
      new ParallelRaceGroup(new WaitCommand(4),
        new IntakeSpinForward(intake),
        new StorageCollect(storage),
        new DriveToPose(base, new Pose2d(-1.2, 0, Rotation2d.fromDegrees(90)))
      ),
      new ParallelRaceGroup(new WaitCommand(4),
        new DriveToPose(base, new Pose2d(0, 0, Rotation2d.fromDegrees(-90))),
        new StorageCollect(storage)
      ),
      new ParallelDeadlineGroup((new WaitCommand(6)),
        new AimWithLimelight(base, camera),
        new FlywheelSpinWithLimelight(flywheel, camera),
        new AutonFeedShot(storage)
      )
    );
  }
}
