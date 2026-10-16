package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class SetInterpolatedShooterRPMSOTM extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private Shooter shooter;
    private Hood hood;

    public SetInterpolatedShooterRPMSOTM(CommandSwerveDrivetrain _drivetrain, Shooter _shooter, Hood _hood){
        drivetrain = _drivetrain;
        shooter = _shooter;
        hood = _hood;

       
        addRequirements(_shooter);
        addRequirements(_hood);
    }
    public void initialize(){
      }
    public void execute(){
           if(DriverStation.getAlliance().orElse(Alliance.Red)==Alliance.Blue){
            shooter.setRPM(shooter.getInterpolatedRPM(drivetrain.distanceToPoseSOTM(Constants.FieldSetpoints.blueHubPose)));
            hood.setSetpointDirect( (shooter.getInterpolatedHoodAngle(drivetrain.distanceToPoseSOTM(Constants.FieldSetpoints.blueHubPose))));
        }else{
        shooter.setRPM(shooter.getInterpolatedRPM(drivetrain.distanceToPoseSOTM(Constants.FieldSetpoints.redHubPose)));
        hood.setSetpointDirect((shooter.getInterpolatedHoodAngle(drivetrain.distanceToPoseSOTM(Constants.FieldSetpoints.redHubPose))));
    }
    }
    
    public boolean isFinished(){
       return false;
    }

    public void end(boolean interrupted){}
}