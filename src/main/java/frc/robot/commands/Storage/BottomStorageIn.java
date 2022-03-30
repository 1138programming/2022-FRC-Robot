package frc.robot.commands.Storage;

import frc.robot.Robot;
import frc.robot.subsystems.Storage;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;

public class BottomStorageIn extends CommandBase {
    private Storage storage;

    public BottomStorageIn(Storage storage){
        this.storage = storage;
        addRequirements(storage);
    }

    public void intialize(){}

    public void execute(){
        storage.moveBottom(kBottomStoragePWM);
    }

    public void end(boolean interrupted){}

    public boolean isFinished(){
        return false;
    }
}
