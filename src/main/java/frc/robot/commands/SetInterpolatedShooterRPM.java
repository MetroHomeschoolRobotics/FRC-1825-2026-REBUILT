package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class SetInterpolatedShooterRPM extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private Shooter shooter;

    public SetInterpolatedShooterRPM(CommandSwerveDrivetrain _drivetrain, Shooter _shooter){
        drivetrain = _drivetrain;
        shooter = _shooter;

        addRequirements(_drivetrain);
        addRequirements(_shooter);
    }
    public void initialize(){
        if(DriverStation.getAlliance().get()==Alliance.Blue){
            shooter.setRPM(shooter.getInterpolatedRPM(drivetrain.distanceToPose(Constants.FieldSetpoints.blueHubPose)));
        }else{
        shooter.setRPM(shooter.getInterpolatedRPM(drivetrain.distanceToPose(Constants.FieldSetpoints.redHubPose)));
    }}
    public void execute(){}
    
    public boolean isFinished(){
        return shooter.atSetpoint();
    }

    public void end(boolean interrupted){}
}
