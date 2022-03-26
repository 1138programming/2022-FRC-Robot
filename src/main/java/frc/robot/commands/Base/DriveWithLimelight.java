// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import static frc.robot.Constants.*;

import frc.robot.Robot;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.NeoBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;

public class DriveWithLimelight extends CommandBase {

  private double fbSpeed; //Speed of the robot in the x direction (forward).
  private double lrSpeed; //Speed of the robot in the Y direction (sideways).
  private double rot;
  private Camera camera;
  private NeoBase base;
  private PIDController rotationController;
  private final double kLimglightP = 1;
  private final double kLimglightI = 0;
  private final double kLimglightD = 0;

  /** Creates a new DriveWithLimelight. */
  public DriveWithLimelight(NeoBase base, Camera camera) {

    this.base = base;
    this.camera = camera;

    rotationController = new PIDController(kLimglightP, kLimglightI, kLimglightD);

    addRequirements(base);
    addRequirements(camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    base.resetAllRelEncoders();
    if (camera.getTargetFound() == false) {
      SmartDashboard.putBoolean("Limelight Target Found", false);
    } 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    camera.LEDOn();
    fbSpeed = (-Robot.robotContainer.getLogiLeftYAxis());
    
    lrSpeed = (Robot.robotContainer.getLogiLeftXAxis());
    
    rot = (-Robot.robotContainer.getLogiRightXAxis());

    rot -= rotationController.calculate(camera.getXOffset()/KLimelightRange, 0);
    
    base.drive(fbSpeed, lrSpeed, rot, true);
    // base.drive(fbSpeed, lrSpeed, rot, true);
    SmartDashboard.putBoolean("Limelight Target Found", camera.getTargetFound());
    
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
