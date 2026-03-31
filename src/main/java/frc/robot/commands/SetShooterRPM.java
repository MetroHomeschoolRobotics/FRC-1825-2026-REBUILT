package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetShooterRPM extends Command{
    private Shooter shooter;
    private double RPM;
    public SetShooterRPM(Shooter _shooter,double _RPM){
        addRequirements(_shooter);
        shooter=_shooter;
        RPM=_RPM;
    }
    public void initialize(){shooter.setRPM(RPM);}
    public void execute(){
        
    }
    public void end(boolean interrupted){}
    public boolean isFinished(){return true;}
}
