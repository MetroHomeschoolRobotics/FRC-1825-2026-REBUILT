package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private TalonFX intake = new TalonFX(Constants.MotorIDs.intakeID);
    private TalonFX intakeRetractor = new TalonFX(Constants.MotorIDs.intakeRetractorID);
    private Mechanism2d intake2d = new Mechanism2d(Units.inchesToMeters(20), Units.inchesToMeters(4));
   
    private final DCMotor TopIndexDCMotors = DCMotor.getKrakenX60Foc(1);
    private final LinearSystem<N2, N1, N2> TopIndexFlywheelSystem = LinearSystemId.createDCMotorSystem(TopIndexDCMotors, 0.0005, 1);
    private final DCMotorSim TopIndexFlywheelSim = new DCMotorSim(TopIndexFlywheelSystem, TopIndexDCMotors);

    public Intake(){}
    public void setIntakeSpeed(double speed){
        intake.set(speed);
    }
    public void stopIntake(){
        intake.set(0);
    }
    public void setRetractorSpeed(double speed){
        intakeRetractor.set(speed);
    }
    public void stopIntakeRetractor(){
        intakeRetractor.set(0);
    }
    public void simulationPeriodic(){
        
    }
}
