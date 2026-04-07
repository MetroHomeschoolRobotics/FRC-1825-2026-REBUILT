package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Turret;
import frc.robot.Constants;

public class directdriveturrret extends Command {
    private Turret turret;
    private CommandXboxController joystickInput;
    public directdriveturrret(Turret _turret,CommandXboxController _joystickInput){
        turret = _turret;
        joystickInput = _joystickInput;
        addRequirements(turret);
    }
    public void initialize(){

    }
    public void execute(){
        //TO/DO check rotation directions later
       turret.set(MathUtil.applyDeadband(-joystickInput.getRightX(),0.10));
        // TODO driven turret disable
    }
    public void end(boolean interrupted){
       turret.set(0);
       // TODO driven turret disable
    }
    public boolean isFinished(){
        return false;
    }
}
