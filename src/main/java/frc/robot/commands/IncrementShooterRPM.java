package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;

public class IncrementShooterRPM extends Command{
    private Shooter shooter;
    private CommandXboxController xbox;
    public IncrementShooterRPM(Shooter _shooter,CommandXboxController _xbox){
        addRequirements(_shooter);
        shooter=_shooter;
        xbox=_xbox;
    }
    public void initialize(){}
    public void execute(){
        shooter.incrementRPM(-xbox.getLeftY());
    }
    public void end(boolean interrupted){}
    public boolean isFinished(){return false;}
}