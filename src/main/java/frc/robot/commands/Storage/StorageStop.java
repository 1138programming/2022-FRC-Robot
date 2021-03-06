// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Storage;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;
import static frc.robot.Constants.*;

public class StorageStop extends CommandBase {
  private Storage storage;

  /** Creates a new StorageStop. */
  public StorageStop(Storage storage) {
    this.storage = storage;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(storage);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //First number is the top motor, second number is the bottom motor
    storage.moveTop(0);
    storage.moveBottom(0);
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
