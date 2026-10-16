package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class AutoSetInterpolatedShooterRPM extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private Shooter shooter;
    private Hood hood;

    public AutoSetInterpolatedShooterRPM(CommandSwerveDrivetrain _drivetrain, Shooter _shooter, Hood _hood){
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
            shooter.setRPM(shooter.getInterpolatedRPM(drivetrain.distanceToPose(Constants.FieldSetpoints.blueHubPose)));
            hood.setSetpointDirect( (shooter.getInterpolatedHoodAngle(drivetrain.distanceToPose(Constants.FieldSetpoints.blueHubPose))));
        }else{
        shooter.setRPM(shooter.getInterpolatedRPM(drivetrain.distanceToPose(Constants.FieldSetpoints.redHubPose)));
        hood.setSetpointDirect((shooter.getInterpolatedHoodAngle(drivetrain.distanceToPose(Constants.FieldSetpoints.redHubPose))));
    }
    SmartDashboard.putNumber("Command distance to Pose", drivetrain.distanceToPose(Constants.FieldSetpoints.redHubPose));
    SmartDashboard.putNumber("Command interpolated hood angle", shooter.getInterpolatedHoodAngle(drivetrain.distanceToPose(Constants.FieldSetpoints.redHubPose)));
    }
    
    public boolean isFinished(){
       return false;
    }

    public void end(boolean interrupted){}
}
