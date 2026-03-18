package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PointToHub extends Command {
    CommandSwerveDrivetrain drivetrain;
    Pose2d hubPose;
    private final SwerveRequest.FieldCentricFacingAngle point = new SwerveRequest.FieldCentricFacingAngle();
    public PointToHub(CommandSwerveDrivetrain _drivetrain){
        drivetrain = _drivetrain;
        addRequirements(drivetrain);
    }
    public void initialize(){
        
    }
    public void execute(){
    }
    public boolean isFinished(){return false;}
    public void end(boolean interrupted){}
}
