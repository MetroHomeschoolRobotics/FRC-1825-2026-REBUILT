package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ChangeTurretMode;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.RunFullIndexing;
import frc.robot.commands.RunIntake;
import frc.robot.commands.AutoSetInterpolatedShooterRPM;
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

        routine.active().onTrue(
            startToShoot.resetOdometry()
            .andThen(new ChangeTurretMode(drivetrain, "Hub"))
            .andThen(startToShoot.cmd()).deadlineFor(new AutoSetInterpolatedShooterRPM(drivetrain,shooter))
            .andThen(Commands.waitSeconds(0.3)).deadlineFor(new RunFullIndexing(indexer))
            .andThen(Commands.waitSeconds(.5)).deadlineFor(new DeployIntake(intake))
            .andThen(shootToDepot.cmd()).raceWith(new AutoSetInterpolatedShooterRPM(drivetrain,shooter), new SequentialCommandGroup(Commands.waitSeconds(3.6),new RunIntake(intake)))
            .andThen(Commands.waitSeconds(5)).deadlineFor(new RunFullIndexing(indexer))
            );


        return routine;
    }
}
