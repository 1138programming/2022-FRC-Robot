// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Base;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Camera;
// import frc.robot.subsystems.NeoBase;

// public class AimWithLimelight extends CommandBase {
//   private Camera camera;
//   private NeoBase base;
//   private PIDController rotationController;

//   public AimWithLimelight(NeoBase base, Camera camera) {
//     this.base = base;
//     this.camera = camera;
//     this.rotationController = new PIDController()

//     addRequirements(base, camera);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
    
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if (camera.getTargetFound() == 0)
//     {
//       rot = 1;
//     }
//     else {

//     }
//     base.drive(0, 0, rot, false);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
