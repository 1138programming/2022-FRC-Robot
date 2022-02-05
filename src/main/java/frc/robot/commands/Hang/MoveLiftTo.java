// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hang;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hang;

public class MoveLiftTo extends CommandBase {
  private Hang hang;
  private double liftSetPoint;
  private double currentEncoderValue;
  private PIDController liftPosController;
  private double kP, kI, kD;
  private double output;

  /** Creates a new MoveArmsTo. */
  public MoveLiftTo(Hang hang, double liftSetPoint) {
    this.hang = hang;
    this.liftSetPoint = liftSetPoint;

    kP = 0.06;
    kI = 0;
    kD = 0;
    liftPosController = new PIDController(kP, kI, kD);
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
    currentEncoderValue = hang.getLevelEncoder();
    output = liftPosController.calculate(currentEncoderValue, liftSetPoint);
    hang.moveLift(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hang.moveLift(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (currentEncoderValue >= liftSetPoint) {
      return true;
    }
    return false;
  }
}
