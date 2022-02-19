// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class IntakeSwivelDownToLimit extends CommandBase {
  private Intake intake;
  /** Creates a new IntakeSwivelDownToLimit. */
  public IntakeSwivelDownToLimit(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.getPixyColorRed() == 1)
    {
      // intake.move(1);
    intake.moveSwivel(KIntakeSwivelPWM);
    }
    else
    {
      // intake.move(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.moveSwivel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (intake.getBottomLimitSwitch()) {
      return true;
    }
    return false;
  }
}
