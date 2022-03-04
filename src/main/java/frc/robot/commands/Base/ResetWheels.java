// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

<<<<<<< HEAD:src/main/java/frc/robot/commands/Flywheel/FlywheelStop.java
package frc.robot.commands.Flywheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import static frc.robot.Constants.*;

public class FlywheelStop extends CommandBase {
  private final Flywheel flywheel;

  public FlywheelStop(Flywheel flywheel) {
    this.flywheel = flywheel;
    addRequirements(flywheel);
=======
package frc.robot.commands.Base;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NeoBase;

public class ResetWheels extends CommandBase {
  private NeoBase base;
  /** Creates a new ResetWheels. */
  public ResetWheels(NeoBase base) {
    this.base = base;
    addRequirements(base);
>>>>>>> MotionProfileTesting:src/main/java/frc/robot/commands/Base/ResetWheels.java
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< HEAD:src/main/java/frc/robot/commands/Flywheel/FlywheelStop.java
    flywheel.move(0);
=======
    base.resetWheelAngles();
>>>>>>> MotionProfileTesting:src/main/java/frc/robot/commands/Base/ResetWheels.java
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return base.getWheelsHavereset();
  }
}
