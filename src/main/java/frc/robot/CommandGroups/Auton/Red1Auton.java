// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Base.RotateToHeading;
import frc.robot.subsystems.NeoBase;

import frc.robot.Trajectories.Red1;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Red1Auton extends SequentialCommandGroup {
  Red1 red1;
  /** Creates a new Red1Auton. */
  public Red1Auton(NeoBase base) {
    // red1 = new Red1(base);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // red1.getTrajectory(1),
      // new RotateToHeading(base, 90),
      // red1.getTrajectory(2)
    );
  }
}
