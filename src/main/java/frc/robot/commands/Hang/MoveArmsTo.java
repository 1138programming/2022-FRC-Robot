// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hang;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hang;

public class MoveArmsTo extends CommandBase {
  private Hang hang;
  private double armSetPoint;
  private double currentEncoderValue;
  private PIDController armPosController;
  private double kP, kI, kD;
  private double output;

  /** Creates a new MoveArmsTo. */
  public MoveArmsTo(Hang hang, double armSetPoint) {
    this.hang = hang;
    this.armSetPoint = armSetPoint;

    kP = 0.06;
    kI = 0;
    kD = 0;
    armPosController = new PIDController(kP, kI, kD);
  }

  // Called when the command is initially scheduled.
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
    hang.moveArms(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hang.moveArms(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (currentEncoderValue >= armSetPoint) {
      return true;
    }
    return false;
  }
}
