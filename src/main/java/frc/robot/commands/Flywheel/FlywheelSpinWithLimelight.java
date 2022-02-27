// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Flywheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Camera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;

public class FlywheelSpinWithLimelight extends CommandBase {
  private final Flywheel flywheel;
  private final Camera camera;
  private double flywheelOutput;
  private double distanceFromHub;
  /** Creates a new FlywheelSpinWithLimelight. */
  public FlywheelSpinWithLimelight(Flywheel flywheel, Camera camera) {
    this.flywheel = flywheel;
    this.camera = camera;
    flywheelOutput = 0;
    distanceFromHub = 0;
    addRequirements(flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //max flywheel RPM is 2896 RPM
    //nolan testing data: 202 in. is 85% (2462 rpm), 60 in. is 65% flywheel speed (1883 rpm)
    //assume linear relationship between distance and RPM required to score, (2462-1883)/(202-60) = 4.077
    distanceFromHub = camera.getDistance();
    if (distanceFromHub > 60) {
      flywheelOutput = 1883 + distanceFromHub * 4.077;
    }
    else {
      flywheelOutput = 0;
    }
    SmartDashboard.putNumber("flywheel output (RPM)", flywheelOutput);
    flywheel.move(flywheelOutput);    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.move(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
