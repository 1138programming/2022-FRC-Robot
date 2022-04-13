// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import frc.robot.Robot;
import frc.robot.subsystems.NeoBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveWithJoysticks extends CommandBase {

  private final NeoBase base;

  private double fbSpeed; //Speed of the robot in the x direction (forward).
  private double lrSpeed; //Speed of the robot in the Y direction (sideways).
  private double rot;

  private PIDController rotationCorrectionPID;
  private double initHeading;

  private double kRotationP = 0.005;
  private double kRotationI = 0;
  private double kRotationD = 0;

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(NeoBase base) {

    this.base = base;
  
    rotationCorrectionPID = new PIDController(kRotationP, kRotationI, kRotationD);

    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    base.resetAllRelEncoders();  
    initHeading = base.getHeadingDeg();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    fbSpeed = (-Robot.robotContainer.getLogiLeftYAxis());
    
    lrSpeed = (Robot.robotContainer.getLogiLeftXAxis());
    
    rot = (-Robot.robotContainer.getLogiRightXAxis());
    
    if (Math.abs(rot) <= 0.01 && (Math.abs(fbSpeed) >= 0.01 || Math.abs(lrSpeed) >= 0.01)){
      rot = rotationCorrectionPID.calculate(base.getHeadingDeg(), initHeading);
    }
    else {
      initHeading = base.getHeadingDeg();
    }
    base.drive(fbSpeed, lrSpeed, rot, true);
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
