package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Storage extends SubsystemBase {
  //Defining motors
  private VictorSPX bottomStorageMotor;
  private VictorSPX topStorageMotor;
  //Defining encoders
  private DigitalInput ballSensorBottom;
  private DigitalInput ballSensorTop;

  //Constructor is constructing
  public Storage(){
    //Storage Motors Init
    bottomStorageMotor = new VictorSPX(KBottomStorageVictor);
    topStorageMotor = new VictorSPX(KTopStorageVictor);
    //Storage Sensor Init
    ballSensorBottom = new DigitalInput(KStorageSensorBottom);
    ballSensorTop = new DigitalInput(KStorageSensorTop);

    //invert motors
    bottomStorageMotor.setInverted(true);
    topStorageMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("storage top limit", getBallSensorTop());
    SmartDashboard.putBoolean("storage bott limit", getBallSensorBottom());
  }

  public void moveTop(double speed) {
    topStorageMotor.set(VictorSPXControlMode.PercentOutput, speed);
  }
  public void moveBottom(double speed) {
    bottomStorageMotor.set(VictorSPXControlMode.PercentOutput, speed);
  }
  public boolean getBallSensorTop(){
    return ballSensorTop.get();
  }
  public boolean getBallSensorBottom(){
    return ballSensorBottom.get();
  }
}