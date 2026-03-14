package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.Trajectory;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ChangeTurretMode;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.RunFullIndexing;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SetInterpolatedShooterRPM;
import frc.robot.commands.SetShooterRPM;
import frc.robot.commands.AutoSetInterpolatedShooterRPM;
import frc.robot.Constants;
import frc.robot.Constants.TurretMode;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class Autos {
    private AutoFactory factory;
    private Turret turret;
    private Hood hood;
    private Intake intake;
    private Indexer indexer;
    private Shooter shooter;
    private CommandSwerveDrivetrain drivetrain;
    public Autos(AutoFactory _factory,Turret _turret, Hood _hood, Intake _intake,Indexer _indexer,Shooter _shooter,CommandSwerveDrivetrain _drivetrain){
        factory = _factory;
        turret = _turret;
        hood = _hood;
        intake = _intake;
        indexer =_indexer;
        shooter = _shooter;
        drivetrain = _drivetrain;
    }
    public AutoRoutine midAuto(){
        AutoRoutine routine = factory.newRoutine("midAuto");

        final AutoTrajectory startToShoot = routine.trajectory("midToShooting");
        final AutoTrajectory shootToDepot = routine.trajectory("midShootingToDepot");

        routine.active().onTrue(startToShoot.resetOdometry()
        .andThen(startToShoot.cmd())
        .andThen(new ParallelDeadlineGroup(shootToDepot.cmd(),new SequentialCommandGroup(new DeployIntake(intake),new RunIntake(intake))))
        .andThen(new SequentialCommandGroup(new ChangeTurretMode(drivetrain, TurretMode.HUB),new SetInterpolatedShooterRPM(drivetrain, shooter)))
        .andThen(new ParallelRaceGroup(new RunFullIndexing(indexer),Commands.waitSeconds(5)))
            .andThen(new SetShooterRPM(shooter,0))
            // startToShoot.resetOdometry()
            // .andThen(new ChangeTurretMode(drivetrain, "Hub"))
            // .andThen(startToShoot.cmd())//.deadlineFor(new AutoSetInterpolatedShooterRPM(drivetrain,shooter))
            // .andThen(Commands.waitSeconds(0.3)).deadlineFor(new RunFullIndexing(indexer))
            // .andThen(Commands.waitSeconds(.5)).deadlineFor(new DeployIntake(intake))
            // .andThen(shootToDepot.cmd())//.raceWith(new AutoSetInterpolatedShooterRPM(drivetrain,shooter), new SequentialCommandGroup(Commands.waitSeconds(3.6),new RunIntake(intake)))
            // .andThen(Commands.waitSeconds(5)).deadlineFor(new RunFullIndexing(indexer))
            );


        return routine;
    }
    public AutoRoutine leftAuto(){
        AutoRoutine routine = factory.newRoutine("leftAuto");
        final AutoTrajectory startToMid = routine.trajectory("leftTrenchToMid");
        final AutoTrajectory midToShoot = routine.trajectory("leftMidToAlliance");
        final var idle = new SwerveRequest.Idle();
        routine.active().onTrue(
                  startToMid.resetOdometry().
            andThen(new ParallelDeadlineGroup(startToMid.cmd(),new SequentialCommandGroup(new DeployIntake(intake),new RunIntake(intake))))
            .andThen(midToShoot.cmd())
            .andThen(new SequentialCommandGroup(new ChangeTurretMode(drivetrain, TurretMode.HUB),new SetInterpolatedShooterRPM(drivetrain, shooter)))
            
            .andThen(new ParallelRaceGroup(new RunFullIndexing(indexer),Commands.waitSeconds(5)))
            .andThen(new SetShooterRPM(shooter,0))

        );
        return routine;
    }
    public AutoRoutine rightAuto(){
        AutoRoutine routine = factory.newRoutine("rightAuto");
        final AutoTrajectory startToMid = routine.trajectory("rightTrenchToMid");
        final AutoTrajectory midToShoot = routine.trajectory("rightMidToAlliance");
        final var idle = new SwerveRequest.Idle();
        routine.active().onTrue(
            // startToMid.resetOdometry()
            // .andThen(startToMid.cmd()).deadlineFor(new SequentialCommandGroup(new DeployIntake(intake),new RunIntake(intake)))
            // .andThen(midToShoot.cmd())
            // .andThen(new SequentialCommandGroup(new ChangeTurretMode(drivetrain, "Hub"),new SetInterpolatedShooterRPM(drivetrain, shooter)))
            // .andThen(drivetrain.applyRequest(() -> idle))
            // .andThen(new RunFullIndexing(indexer)).raceWith(Commands.waitSeconds(5))
            // .andThen(new SetShooterRPM(shooter,0))
            startToMid.resetOdometry().
            andThen(new ParallelDeadlineGroup(startToMid.cmd(),new SequentialCommandGroup(new DeployIntake(intake),new RunIntake(intake))))
            .andThen(midToShoot.cmd())
            .andThen(new SequentialCommandGroup(new ChangeTurretMode(drivetrain, TurretMode.HUB),new SetInterpolatedShooterRPM(drivetrain, shooter)))
            
            .andThen(new ParallelRaceGroup(new RunFullIndexing(indexer),Commands.waitSeconds(5))).andThen(new SetShooterRPM(shooter,0))
        );
        return routine;
    }
}
