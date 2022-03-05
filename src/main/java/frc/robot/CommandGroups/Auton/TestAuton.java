// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Base.DriveToPose;
import frc.robot.commands.Base.ResetGyro;
import frc.robot.commands.Base.ResetOdometry;
import frc.robot.subsystems.NeoBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuton extends SequentialCommandGroup {
  /** Creates a new TestAuton. */
  public TestAuton(NeoBase base) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetOdometry(base),
      new ResetGyro(base),

      new DriveToPose(base, new Pose2d(2, 0, new Rotation2d())),

      new DriveToPose(base, new Pose2d(0, 1.25, new Rotation2d(Math.toRadians(90)))),

      new DriveToPose(base, new Pose2d(1, 1, new Rotation2d(Math.toRadians(-90))))
    );
  }
}
