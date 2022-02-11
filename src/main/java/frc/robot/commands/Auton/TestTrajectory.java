// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import java.util.Arrays;
import java.util.function.Consumer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.subsystems.NeoBase;

public class TestTrajectory extends CommandBase {
  TrajectoryConfig config;
  Trajectory trajectory;
  NeoBase base;
  SwerveControllerCommand command;
  

  public TestTrajectory(NeoBase base) {
    this.base = base;
    config = new TrajectoryConfig(1, 1);
    config.setKinematics(base.getKinematics());
    trajectory = TrajectoryGenerator.generateTrajectory(
      Arrays.asList(new Pose2d(), new Pose2d(1, 0, new Rotation2d())),
      config
    ); 
    // command = new SwerveControllerCommand(trajectory, pose, kinematics, xController, yController, thetaController, desiredRotation, outputModuleStates, requirements)
    command = new SwerveControllerCommand(
      trajectory, 
      base::getPose, 
      base.getKinematics(), 
      new PIDController(0.047116, 0, 0),
      new PIDController(0.047116, 0, 0),
      new ProfiledPIDController (0.69, 0, 0, new TrapezoidProfile.Constraints(1, 1)),
      base::setModuleStates,
      base
    );
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    command.execute();
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    base.drive(0, 0, 0, false);
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (command.isFinished()) {
      return true;
    }
    return false;
  }
}
