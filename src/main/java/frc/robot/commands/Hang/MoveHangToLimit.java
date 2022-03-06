// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hang;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hang;
import static frc.robot.Constants.*;

public class MoveHangToLimit extends CommandBase {
  private Hang hang;
  /** Creates a new MoveHangToLimit. */
  public MoveHangToLimit(Hang hang) {
    this.hang = hang;
    addRequirements(hang);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hang.moveLeveHangUnrestricted(KLevelHangSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hang.getHangLimit();
  }
}
