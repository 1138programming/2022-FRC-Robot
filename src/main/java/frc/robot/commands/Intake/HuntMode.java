// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;


public class HuntMode extends CommandBase {
  /** Creates a new HuntMode. */
  private final Intake intake;
  public HuntMode(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.swivelToPos(KIntakeAngle); // Sets the intake to hunt mode.
    SmartDashboard.putNumber("Pixy", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {   
    
    
    
    
    ArrayList<Block> blocks = intake.().getPixy().getCCC().getBlocks();
		if (blocks == null) {
			System.err.println("No Blocks");
			return;






  if (intake.getIntakeEncoderDeg() == KIntakeAngle) //Checks to see if the the intake is at the correct angle.
    {  
      if (intake.getPixyColorRed() >= 1 || intake.getPixyColorBlue() == 3) 
      /* Checks to see if the ball is in the intake by adding the bumper objects to the total count of objects. 
         Then it caps the amount of objects it will register to 3, making it so the code does not need to be switched before the match.
      */
        {
          intake.swivelToPos(10);
          intake.moveSpin(KIntakeSpinPWM); 
          // This swivels the intake to the collection position and spins the intake motor.
        }
      else
      {
        intake.swivelToPos(KIntakeAngle);
        intake.moveSpin(0);
        // This keeps the intake in the hunt postion and stops the motor.
      }
      if (intake.getBottomLimitSwitch()) 
      {
        intake.moveSwivel(0);
        // Stops the intakes when it gets to the limit switch. This is a failsafe if the intake starts to go too far down. 
      }
    }
  else
    {
      if (intake.getIntakeEncoderDeg() <= KIntakeAngle && (intake.getPixyColorRed() == 1 || intake.getPixyColorBlue() == 3))
        // Checks to see if the pixy still sees the ball and if the intake is below hunt mode angle.
       {
        intake.moveSpin(KIntakeSpinPWM);
        // Spins the intake.
       }
      else
       {
         intake.swivelToPos(KIntakeAngle);
         // Moves the intake back to hunt mode. 
       }

    }
    SmartDashboard.putNumber("Pixy", intake.getPixyColorRed());
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
