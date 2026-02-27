package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class SetTurretAngle extends Command{
    private Turret turret;
    private double angle;
    public SetTurretAngle(Turret _turret,double _angle){
        turret = _turret;
        angle = _angle;
        addRequirements(_turret);
    }
    public void initialize(){}
    public void execute(){
        turret.turretSetSetpoint(angle);;
    }
    public boolean isFinished(){
        return false;
    }
}