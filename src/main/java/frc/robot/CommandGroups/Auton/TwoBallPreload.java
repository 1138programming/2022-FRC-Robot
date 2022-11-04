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
public class TwoBallPreload extends SequentialCommandGroup {
  public TwoBallPreload(NeoBase base, Camera camera, Storage storage, Intake intake, Flywheel flywheel) {
    addCommands(
      new ResetOdometry(base),
      new ResetGyro(base),
      new ParallelDeadlineGroup(new WaitCommand(2),
        new StowedMode(intake)
      ),

      // new ParallelDeadlineGroup(new WaitCommand(2),
      //   new FlywheelSpinAtRPM(flywheel, 1900),
      //   new StowedMode(intake)
      // ),
      
      new ParallelDeadlineGroup(new WaitCommand(5),
        new FlywheelSpinAtRPM(flywheel, 1900),
        new ParallelRaceGroup(new WaitCommand(1.5),
          new StorageSpinIntoFlywheel(storage))
      ),

      new ResetOdometry(base),
      new ResetGyro(base),
      new ParallelRaceGroup(new WaitCommand(3),
        new DriveToPose(base, new Pose2d(-1.06, 0, Rotation2d.fromDegrees(0)))
      )
    );
    }
  }
  