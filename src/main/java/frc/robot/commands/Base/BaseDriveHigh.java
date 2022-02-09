// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import static frc.robot.Constants.*;
import frc.robot.subsystems.NeoBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BaseDriveHigh extends CommandBase {

  private final NeoBase base;

  /** Creates a new BaseDriveHigh. */
  public BaseDriveHigh(NeoBase base) {
    this.base = base;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    base.setMaxDriveSpeed(kBaseDriveMediumSpeed); //set to medium for testing
    // base.setMaxDriveSpeed(kBaseDriveHighSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
