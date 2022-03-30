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
import frc.robot.commands.Flywheel.FlywheelAutonSpin;
import frc.robot.commands.Flywheel.FlywheelSpinWithLimelight;
import frc.robot.commands.Intake.StowedMode;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NeoBase;
import frc.robot.subsystems.Storage;

/*
Auton Setup:  Robot can be setup anywhere in the tarmac,
              Robot should be as close to the back line as possible and aimed at the goal. 
*/
public class DriveBackAndShoot extends SequentialCommandGroup {
  public DriveBackAndShoot(NeoBase base, Camera camera, Storage storage, Intake intake, Flywheel flywheel) {
    addCommands(

      //move intake slightly down to avoid contact with flywheel
      new ParallelRaceGroup(new WaitCommand(0.3),
        new StowedMode(intake)
      ),

      // flywheel spinup and shoot for 3 secs
      new ParallelRaceGroup(
        new WaitCommand(3),
        new AutonFeedShot(storage),
        new FlywheelAutonSpin(flywheel, 1950)
      ),

      //base move back 1.5 m
      new ResetGyro(base),
      new ResetOdometry(base),
      new ParallelRaceGroup( //timer used to prevent the base from doing somehting really bad if something is wrong
        new WaitCommand(4),
        new DriveToPose(base, new Pose2d(-1.5, 0, Rotation2d.fromDegrees((0))))
      )
    );
  }
}
