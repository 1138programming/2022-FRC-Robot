// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

<<<<<<< HEAD:src/main/java/frc/robot/commands/Hang/MoveRachetOut.java
package frc.robot.commands.Hang;

import frc.robot.Robot;

import static frc.robot.Constants.*;
import frc.robot.subsystems.Hang;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveRachetOut extends CommandBase {
  private Hang hang;
  /** Creates a new MoveClawOut. */
  public MoveRachetOut(Hang hang) {
    this.hang = hang;
=======
package frc.robot.commands.Base;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NeoBase;

public class BaseStop extends CommandBase {
  NeoBase base;
  /** Creates a new BaseStop. */
  public BaseStop(NeoBase base) {
    this.base = base;
    addRequirements(base);
>>>>>>> MotionProfileTesting:src/main/java/frc/robot/commands/Base/BaseStop.java
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    base.drive(0, 0, 0, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< HEAD:src/main/java/frc/robot/commands/Hang/MoveRachetOut.java
    hang.moveRachet(1);
=======
>>>>>>> MotionProfileTesting:src/main/java/frc/robot/commands/Base/BaseStop.java
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
