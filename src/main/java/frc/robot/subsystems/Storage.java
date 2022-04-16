package frc.robot.subsystems;

import static frc.robot.Constants.*;
//ctre
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
//wpilib
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Storage extends SubsystemBase {

  private VictorSPX bottomStorageMotor;
  private VictorSPX topStorageMotor;

  private DigitalInput ballSensorBottom;
  private DigitalInput ballSensorTop;

  public Storage() {

    bottomStorageMotor = new VictorSPX(KBottomStorageVictor);
    topStorageMotor = new VictorSPX(KTopStorageVictor);

    ballSensorBottom = new DigitalInput(KStorageSensorBottom);
    ballSensorTop = new DigitalInput(KStorageSensorTop);

    bottomStorageMotor.setInverted(false);
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
  public boolean getBallSensorTop() {
    return ballSensorTop.get();
  }
  public boolean getBallSensorBottom() {
    return ballSensorBottom.get();
  }
}