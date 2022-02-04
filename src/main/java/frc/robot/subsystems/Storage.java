package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;;

public class Storage extends SubsystemBase {
  private CANSparkMax storageMotor;
    
  public Storage(){
    storageMotor = new CANSparkMax(KStorageSpark, MotorType.kBrushless);
  }
//    
  public void move(double speed) {
    storageMotor.set(speed);
  }
}