// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hang;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hang;
import static frc.robot.Constants.*;

public class MoveArmsToLimit extends CommandBase {
  private Hang hang;
  private double speed;

  /** Creates a new MoveArmsToLimit. */
  public MoveArmsToLimit(Hang hang) {
    this.hang = hang;
    addRequirements(hang);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hang.moveToPosition(KArmsLimit, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(hang.getArmsLimitSwitch()) {
      return true;
    }
    return false;
  }
}
