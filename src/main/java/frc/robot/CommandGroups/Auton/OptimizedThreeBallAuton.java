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
              Line up robot on right tarmac so that it is aimed the goal,
              robot should be as close to the far right corner as possible while still aimed at the goal.
*/
public class OptimizedThreeBallAuton extends SequentialCommandGroup {
  public OptimizedThreeBallAuton(NeoBase base, Camera camera, Storage storage, Intake intake, Flywheel flywheel) {
    addCommands(
      
      // new ParallelRaceGroup(new WaitCommand(0.6),
      //   new StowedMode(intake)
      // ),
      
      // new ParallelRaceGroup(new WaitCommand(3.5),
      //   new FlywheelSpinAtRPM(flywheel, 1900),
      //   new ParallelRaceGroup(new WaitCommand(1.5),
      //     new StorageSpinIntoFlywheel(storage))
      // ),
      
      new ResetGyro(base),
      new ResetOdometry(base),
      new ParallelRaceGroup(new WaitCommand(3),
        new DriveToPose(base, new Pose2d(-1.1, -1, Rotation2d.fromDegrees(-162))),
        new HuntMode(intake),
        new ParallelCommandGroup(
          new IntakeSpinForward(intake),
          new StorageCollect(storage)
        ) 
      )
      
    //   new ResetGyro(base),
    //   new ResetOdometry(base),
    //   new ParallelRaceGroup(new WaitCommand(3),
    //     new StorageCollect(storage),
    //     new IntakeSpinForward(intake),
    //     new DriveToPose(base, new Pose2d(0, 2.73, Rotation2d.fromDegrees(85)))
    //   ),
      
    //   new ResetGyro(base),
    //   new ResetOdometry(base),
    //   new ParallelRaceGroup(new WaitCommand(1.5),
    //     new IntakeSpinForward(intake),
    //     new StorageCollect(storage),
    //     new DriveToPose(base, new Pose2d(-0.5, 1.5, Rotation2d.fromDegrees(103)))
    //   ),

    //   new AimWithLimelight(base, camera),
    //   new ParallelRaceGroup(new WaitCommand(3.5),
    //     new FlywheelSpinWithLimelight(flywheel, camera),
    //     new AutonFeedShot(storage)
    //   ),
        
    //   new ResetGyro(base),
    //   new ResetOdometry(base)
    );
    }
  }
  