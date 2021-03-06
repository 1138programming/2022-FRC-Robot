// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hang;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hang;
import static frc.robot.Constants.*;

public class MoveClawTo extends CommandBase {
  private Hang hang;
  private double pos;

  /** Creates a new ClawOut. */
  public MoveClawTo(Hang hang, double pos) {
    this.pos = pos;
    this.hang = hang;
    addRequirements(hang);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hang.moveClaw(pos);    
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
