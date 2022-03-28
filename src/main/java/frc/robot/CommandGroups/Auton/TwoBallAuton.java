// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.commands.Flywheel.FlywheelAutonSpin;
import frc.robot.commands.Flywheel.FlywheelSpinWithLimelight;
import frc.robot.commands.Intake.HuntMode;
import frc.robot.commands.Intake.IntakeSpinForward;
import frc.robot.commands.Intake.IntakeSwivelDown;
import frc.robot.commands.Intake.StowedMode;
import frc.robot.commands.Storage.StorageCollect;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NeoBase;
import frc.robot.subsystems.Storage;

// NOTE: Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAuton extends SequentialCommandGroup {
  public TwoBallAuton(NeoBase base, Camera camera, Storage storage, Intake intake, Flywheel flywheel) {
    addCommands(
      
      
      new ResetGyro(base),
      new ResetOdometry(base),
      new ParallelRaceGroup(new WaitCommand(2),
        new DriveToPose(base, new Pose2d(1.4, 0, Rotation2d.fromDegrees(0))),
        new HuntMode(intake),
        new StorageCollect(storage),
        new IntakeSpinForward(intake)
      ),

      new ParallelRaceGroup(new WaitCommand(1),
        new StorageCollect(storage),
        new IntakeSpinForward(intake)
      ),
      
      new ResetGyro(base),
      new ResetOdometry(base),
      new ParallelRaceGroup(new WaitCommand(1.5),
        new IntakeSpinForward(intake),
        new StorageCollect(storage),
        new DriveToPose(base, new Pose2d(0, 0, Rotation2d.fromDegrees(180)))
      ),
      new ParallelRaceGroup(new WaitCommand(1),
        new StowedMode(intake),
        new AimWithLimelight(base, camera)
      ),

      new ResetGyro(base),
      new ResetOdometry(base),
      new ParallelRaceGroup(new WaitCommand(1.5),
        new DriveToPose(base, new Pose2d(1.5, 0, Rotation2d.fromDegrees(0)))
      ),
      
      new ParallelRaceGroup(new WaitCommand(6),
      new FlywheelAutonSpin(flywheel, 1950),
      new AutonFeedShot(storage)
      ),

      new ResetGyro(base),
      new ResetOdometry(base),
      new ParallelRaceGroup(new WaitCommand(1.5),
        new DriveToPose(base, new Pose2d(-1.5, 0, Rotation2d.fromDegrees(0)))
      )
    );
  }
}
