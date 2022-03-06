// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hang;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hang;

public class MoveArmsToPosition extends CommandBase {
  private Hang hang;
  private double position;
  /** Creates a new MoveArmsToPosition. */
  public MoveArmsToPosition(Hang hang, double position) {
    this.hang = hang;
    this.position = position;
    addRequirements(hang);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    hang.moveArmsToPosition(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hang.getLeftArmEncoder() <= position + 1 &&
            hang.getLeftArmEncoder() >= position - 1 && 
            hang.getRightArmEncoder() <= position + 1 && 
            hang.getRightArmEncoder() >= position - 1;
  }
}
