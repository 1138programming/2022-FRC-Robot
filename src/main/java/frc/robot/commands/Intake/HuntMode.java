// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.links.Link;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.Pixy2.LinkType;
import io.github.pseudoresonance.pixy2api.links.SPILink;
import io.github.pseudoresonance.pixy2api.*;

public class HuntMode extends CommandBase {
  /** Creates a new HuntMode. */
  private final Intake intake;
  private int state = -1;
  private boolean isCamera = false ;
  public HuntMode(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  
    intake.swivelToPos(KIntakePos); // Sets the intake to hunt mode.
    // intake.swivelToPos(kStowedPos); // Sets the intake to hunt mode.

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.moveSwivel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return (Math.abs(intake.getIntakeEncoderRaw() - 1000) < 50);
    return Math.abs(intake.getIntakeEncoderRaw() - KIntakePos) < 100 || intake.getBottomLimitSwitch();
    // return false;
  }
}
