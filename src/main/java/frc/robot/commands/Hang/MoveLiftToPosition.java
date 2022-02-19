// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hang;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hang;
import static frc.robot.Constants.*;

public class MoveLiftToPosition extends CommandBase {
  private Hang hang;
  private double liftSetPoint;
  private double currentEncoderValue;
  private PIDController liftPosController;
  private double kP, kI, kD;
  private double output;

  /** Creates a new MoveArmsTo. */
  public MoveLiftToPosition(Hang hang) {
    this.hang = hang;
    liftPosController = new PIDController(kP, kI, kD);
    addRequirements(hang);
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
    hang.moveLevel(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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
