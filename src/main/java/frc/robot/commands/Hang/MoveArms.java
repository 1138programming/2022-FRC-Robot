// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hang;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hang;
import static frc.robot.Constants.*;

public class MoveArms extends CommandBase {
  private Hang hang;
  private double armSetPoint;
  private double currentEncoderValue;
  private PIDController armPosController;
  private double kP, kI, kD;
  private double output;

  /** Creates a new MoveArmsTo. */
  public MoveArms(Hang hang) {
    this.hang = hang;
    armPosController = new PIDController(kP, kI, kD);
    addRequirements(hang);
  }

  // Called when the command is initially scheduled.
  // Checks to see if Hang has reset
  @Override
  public void initialize() {
    // if (!hang.getArmHasReset()) {
    //   hang.setArmHasReset(true);
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentEncoderValue = hang.getLeftArmEncoder();
    output = armPosController.calculate(currentEncoderValue, armSetPoint);
    hang.moveToPosition(KLevelPosition, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hang.move(KArmSpeed, 0);
  }

  // Returns true when the command should end.
  // Ends the command when the current encoder value is equal to the arm's set point
  @Override
  public boolean isFinished() {
  //   if (currentEncoderValue >= armSetPoint) {
  //     return true;
    // }
    return false;
  }
}
