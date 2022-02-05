package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;;

public class Storage extends SubsystemBase {
  private CANSparkMax LeftStorageMotor;
  private CANSparkMax RightStorageMotor;
    
  public Storage(){
    LeftStorageMotor = new CANSparkMax(KLeftStorageSpark, MotorType.kBrushless);
    RightStorageMotor = new CANSparkMax(KRightStorageSpark, MotorType.kBrushless);
  }
//    
  public void move(double speed) {
    LeftStorageMotor.set(speed);
    RightStorageMotor.set(speed);
  }
}