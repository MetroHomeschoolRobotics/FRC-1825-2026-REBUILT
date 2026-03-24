package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Turret;

public class IncrementTurretAngle extends Command {
    Turret turret;
    CommandXboxController xbox;
    public IncrementTurretAngle(Turret _Turret, CommandXboxController _xbox){
        turret = _Turret;
        xbox = _xbox;
        addRequirements(_Turret);
    }
    public void initialize(){}
    public void execute(){
        turret.incrementTurretAngle(MathUtil.applyDeadband(xbox.getRightX(),0.10)*1);
    }
    public boolean isFinished(){
        return false;
    }
    public void end(boolean interrupted){}
}
