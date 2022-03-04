// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NeoBase;

public class MoveBase extends CommandBase {
  private NeoBase base;
  private double fbDistance, lrDistance, rot;
  private PIDController fbController, lrController, rotController;

  private double fbSpeed, lrSpeed, rotSpeed;

  /** Creates a new MoveBase. */
  public MoveBase(NeoBase base, double fbDistance, double lrDistance, double rot) {
    this.base = base;
    this.fbDistance = fbDistance;
    this.lrDistance = lrDistance;
    this.rot = rot;
    fbController = new PIDController(1, 0, 0);
    lrController = new PIDController(1, 0, 0);
    rotController = new PIDController(1, 0, 0);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    base.resetAllRelEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // fbSpeed = fbController.calculate(base.get)
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
