// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Miscellaneous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DeadlineTimer extends CommandBase {
  double currentTime;
  double length;

  public DeadlineTimer(double timerLength) {
    length = timerLength + System.currentTimeMillis();
    currentTime = System.currentTimeMillis();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime += (System.currentTimeMillis() - currentTime);
    SmartDashboard.putNumber("current time", currentTime);
    SmartDashboard.putNumber("length", length);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentTime >= length;
  }
}
