// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CommandGroups.AutonFeedShot;
import frc.robot.commands.Base.AimWithLimelight;
import frc.robot.commands.Base.DriveToPose;
import frc.robot.commands.Base.ResetGyro;
import frc.robot.commands.Base.ResetOdometry;
import frc.robot.commands.Flywheel.FlywheelSpinAtRPM;
import frc.robot.commands.Flywheel.FlywheelSpinWithLimelight;
import frc.robot.commands.Intake.HuntMode;
import frc.robot.commands.Intake.IntakeSpinForward;
import frc.robot.commands.Intake.IntakeSwivelDown;
import frc.robot.commands.Intake.StowedMode;
import frc.robot.commands.Storage.StorageCollect;
import frc.robot.commands.Storage.StorageSpinIntoFlywheel;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NeoBase;
import frc.robot.subsystems.Storage;

/*
Auton Setup:  Robot should start in far right corner of the tarmac,
              Line up robot on right tarmac so that it's back bumper lines up with the back tarmac line',
              robot should be as close to the far right corner as possible while still aimed at the goal.
*/
public class OptimizedFiveBallAuton extends SequentialCommandGroup {
  public OptimizedFiveBallAuton(NeoBase base, Camera camera, Storage storage, Intake intake, Flywheel flywheel) {
    addCommands(
            
      new ResetGyro(base),
      new ResetOdometry(base),

      new ParallelDeadlineGroup(new WaitCommand(0.5),
        new FlywheelSpinAtRPM(flywheel, 1900),
        new AimWithLimelight(base, camera)
        // new StowedMode(intake)
      ),
      
      new ParallelRaceGroup(new WaitCommand(0.8),
        new FlywheelSpinWithLimelight(flywheel, camera),
        new StorageSpinIntoFlywheel(storage)
      ),

      new ParallelRaceGroup(new WaitCommand(1.4),
        new FlywheelSpinAtRPM(flywheel, 1950),
        // new HuntMode(intake),
        // new ParallelCommandGroup(
          // new IntakeSpinForward(intake),
          // new StorageCollect(storage),
          // ),
        new DriveToPose(base, new Pose2d(-1, -0.8, Rotation2d.fromDegrees(-147)))
      ),
      
      new ResetGyro(base),
      new ResetOdometry(base),
      new ParallelRaceGroup(new WaitCommand(1.8),
        new FlywheelSpinAtRPM(flywheel, 1950),
        // new StorageCollect(storage),
        // new IntakeSpinForward(intake),
        new DriveToPose(base, new Pose2d(0.7, 2.44, Rotation2d.fromDegrees(82)))
      ),

      new ResetGyro(base),
      new ResetOdometry(base),
      new ParallelRaceGroup(new WaitCommand(2),
        new FlywheelSpinAtRPM(flywheel, 1950),
        // new IntakeSpinForward(intake),
        // new StorageCollect(storage),
        new DriveToPose(base, new Pose2d(-1.38, 1, Rotation2d.fromDegrees(77)))
      ),
      
      new AimWithLimelight(base, camera),
      new ParallelDeadlineGroup(new WaitCommand(1),
        new FlywheelSpinWithLimelight(flywheel, camera),
        new AutonFeedShot(storage)
      ),

      new ResetGyro(base),
      new ResetOdometry(base),
      new ParallelDeadlineGroup(new WaitCommand(4),
        new FlywheelSpinAtRPM(flywheel, 1950),
        // new IntakeSpinForward(intake),
        // new StorageCollect(storage),
        new DriveToPose(base, new Pose2d(-1.05, -5.1, Rotation2d.fromDegrees(-125)))
      ),

      new ResetGyro(base),
      new ResetOdometry(base),
      new ParallelRaceGroup(new WaitCommand(2.6),
        new FlywheelSpinAtRPM(flywheel, 1950),
        // new IntakeSpinForward(intake),
        // new StorageCollect(storage),
        new DriveToPose(base, new Pose2d(-4.4, -0.5, Rotation2d.fromDegrees(153)))
      ),

      new ParallelCommandGroup(
        new AimWithLimelight(base, camera),
        new ParallelRaceGroup(new WaitCommand(2),
          new StorageSpinIntoFlywheel(storage)),
          new FlywheelSpinWithLimelight(flywheel, camera)
        )
      );
    }
  }
  