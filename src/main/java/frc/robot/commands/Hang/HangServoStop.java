// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hang;

import frc.robot.Robot;
import frc.robot.subsystems.Hang;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;


public class HangServoStop extends CommandBase {
  /** Creates a new HangStop. */

  private final Hang hang;

  public HangServoStop(Hang hang) {
    this.hang = hang;
    addRequirements(hang);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hang.move(0, 0);
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
