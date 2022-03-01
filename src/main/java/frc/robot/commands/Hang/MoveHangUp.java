// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hang;

import frc.robot.Robot;

import static frc.robot.Constants.*;
import frc.robot.subsystems.Hang;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class MoveHangUp extends CommandBase {
  private Hang hang;
  /** Creates a new MoveHangUp. */
  public MoveHangUp(Hang hang) {
    this.hang = hang;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // hang.moveLevelHangSpeed(KLevelHangSpeed);
    hang.moveLeveHangUnrestricted(KLevelHangSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
