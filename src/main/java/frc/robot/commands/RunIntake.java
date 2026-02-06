package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
    private Intake intake;
    public RunIntake(Intake _intake){
        addRequirements(_intake);
        intake=_intake;
    }
    public void initialize(){}
    public void execute(){
        intake.setIntakeSpeed(0.5);
    }
    public boolean isFinished(){return false;}
    public void end(boolean interrupted){
        intake.stopIntake();
    }
}
