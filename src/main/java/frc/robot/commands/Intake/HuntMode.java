// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HuntMode extends CommandBase {
  /** Creates a new HuntMode. */
  private final Intake intake;
  public HuntMode(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.swivelToPos(kIntakeHuntPos); // Sets the intake to hunt mode.
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  
    intake.swivelToPos(kIntakeHuntPos); // Sets the intake to hunt mode.
    intake.moveSpin(KIntakeSpinPWM);

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.moveSwivel(0);
    intake.moveSpin(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return (Math.abs(intake.getIntakeEncoderRaw() - 1000) < 50);
    // return Math.abs(intake.getIntakeEncoderRaw() - KIntakePos) < 100 || intake.getBottomLimitSwitch();
    return false;
  }
}
