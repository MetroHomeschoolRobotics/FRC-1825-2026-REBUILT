package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

public class SetHoodAngle extends Command{
    private Hood hood;
    private double angle;
    public SetHoodAngle(Hood _hood,double _angle){
        hood = _hood;
        angle = _angle;
        addRequirements(_hood);
    }
    public void initialize(){hood.setPID(angle);}
    public void execute(){
        
    }
    public boolean isFinished(){
        return true;
    }
}
