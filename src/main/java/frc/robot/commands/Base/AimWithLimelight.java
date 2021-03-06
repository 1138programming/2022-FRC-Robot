// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.NeoBase;

public class AimWithLimelight extends CommandBase {
  private final Camera camera;
  private final NeoBase base;
  private PIDController rotationController;
  private double rot;
  private final double kLimglightP = 1;
  private final double kLimglightI = 0;
  private final double kLimglightD = 0;  

  public AimWithLimelight(NeoBase base, Camera camera) {
    this.base = base;
    this.camera = camera;
    rotationController = new PIDController(kLimglightP, kLimglightI, kLimglightD);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (camera.getTargetFound() == false) {
      SmartDashboard.putBoolean("Limelight Target Found", false);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    camera.LEDOn();
    rot = rotationController.calculate(camera.getXOffset()/KLimelightRange, 0);
    base.drive(0, 0, -rot, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    base.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (camera.getXOffset() < kXOffsetDeadzone && camera.getXOffset() > -kXOffsetDeadzone);
  }
}
