// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Gains;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.NeoBase;

public class AimWithLimelight extends CommandBase {
  private Camera camera;
  private NeoBase base;
  private PIDController rotationController;
  private double rot;
  private Gains pidGains;

  public AimWithLimelight(NeoBase base, Camera camera) {
    this.base = base;
    this.camera = camera;
    pidGains = new Gains(0.9, 0, 0, 0);
    rotationController = new PIDController(pidGains.kP, pidGains.kI, pidGains.kD);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (camera.getTargetFound() == 0) {
      SmartDashboard.putBoolean("Target Found", false);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rot = rotationController.calculate(camera.getXOffset()/KLimelightRange, 0);
    base.drive(0, 0, -rot, false);
    SmartDashboard.putNumber("Rotation", rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    base.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (camera.getXOffset() < kXOffsetDeadzone && camera.getXOffset() > -kXOffsetDeadzone) {
    //   return true;
    // }
    return false;
  }
}