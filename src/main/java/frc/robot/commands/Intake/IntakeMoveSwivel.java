// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import static frc.robot.Constants.*;

public class IntakeMoveSwivel extends CommandBase {
  private Intake intake;
  private boolean direction;
  /** Creates a new IntakeSwivelDown. */
  public IntakeMoveSwivel(Intake intake, boolean direction) {
    this.intake = intake;
    this.direction = direction;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (direction) {
      intake.moveSwivel(-KIntakeSwivelPWM);
    }
    else {
      intake.moveSwivel(KIntakeSwivelPWM);
    }
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
