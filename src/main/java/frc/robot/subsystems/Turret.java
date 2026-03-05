package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.driveToPose;

public class Turret extends SubsystemBase {
    private static CANcoder angle = new CANcoder(Constants.MotorIDs.turretCANID);
        private TalonFX turret = new TalonFX(Constants.MotorIDs.turretMotorID);
        private DigitalInput beambreak = new DigitalInput(0);
        private static double robotAngle;
        private static double rotationCount = 0;
        private static int rotationCountInt = 0;
    
        
        private static PIDController pid = new PIDController(0.0007,.00, 0.00);

         private final Mechanism2d leaderMotorMech2d = new Mechanism2d(2, 2);
    private final MechanismLigament2d leaderMotorFlywheelMech2d = leaderMotorMech2d.getRoot("Flywheel Root leaderMotor", 1, 1)
        .append(new MechanismLigament2d("Flywheel leaderMotor", 1, 0));

     private final StatusSignal<AngularVelocity> leaderMotorVelocity = turret.getVelocity(false);
    private static final double kGearRatio = 1;
        private final DCMotor leaderMotorDCMotors = DCMotor.getKrakenX60Foc(2);
    private final LinearSystem<N2, N1, N2> leaderMotorTurretSystem = LinearSystemId.createDCMotorSystem(leaderMotorDCMotors, 0.004, kGearRatio);
    private final DCMotorSim leaderMotorTurretSim = new DCMotorSim(leaderMotorTurretSystem, leaderMotorDCMotors);

    private final CANcoderSimState angleSim = new CANcoderSimState(angle);


    private static final double kSimLoopPeriod = 0.002; // 2 ms
    private Notifier simNotifier = null;
    private double lastSimTime = 0.0;
        public Turret(){
            startSimThread();
        }
        public static void turretSetSetpoint(double setpoint){
            if(setpoint>180){
                setpoint-=360;
            }else if(setpoint<-180){
                setpoint+=360;
            }
            pid.setSetpoint(setpoint);
            
            
        }
        public static void setRobotAngle(double angle){
            robotAngle = angle+90;
        }
        /**use the cancoder position with the 10:1 gear ratio to get the actual angle,
         * it also adds the robot angle
         * 
         * 
         */
        public double getGearedAngle(){
            //360/10 cuz gear ratios 
            double output; //= angle.getAbsolutePosition().getValueAsDouble()*36;
            // if(output>18){
            //     output-=36;
            // }else if(output<-18){
            //     output+=36;
            // }
            output =rotationCount*36;
            return output+robotAngle;

        }
        public void setPID(double angle){
            pid.setSetpoint(angle);
        }
        public static double getAbsoluteAngle(){
            double output =angle.getAbsolutePosition().getValueAsDouble();
           
            if(output>180){
                output-=360;
                // rotationCount+=0.5;
            }else if(output<-180){
                
                output+=360;
                // rotationCount-=0.5;
            }
            return output;
    }
    public void incrementTurretAngle(double input){
        pid.setSetpoint(pid.getSetpoint()+ input);
    }
    public void fixSetpoint(){
        double setpoint = pid.getSetpoint();
        if(setpoint>180){
                setpoint-=360;
            }else if(setpoint<-180){
                setpoint+=360;
            }
        pid.setSetpoint(setpoint);
    }
    public void periodic(){
        //DOES THIS WORK, IDK
        if(!beambreak.get()){
            angle.setPosition(-5);
        }
       rotationCount=angle.getPosition().getValueAsDouble();
       
       fixSetpoint();
        double output = pid.calculate(getGearedAngle());
        turret.set(output);



        SmartDashboard.putNumber("absolute angle", getAbsoluteAngle());
        SmartDashboard.putNumber("rotation count", rotationCountInt);
        SmartDashboard.putNumber("geared angle", getGearedAngle());
        SmartDashboard.putNumber("setpoint turret", pid.getSetpoint());

    }
    public void simulationPeriodic(){
        angleSim.setVelocity(leaderMotorTurretSim.getAngularVelocityRPM()/60);
        angleSim.addPosition((leaderMotorTurretSim.getAngularVelocityRPM()/60)/50);
        double output = pid.calculate(getGearedAngle());
        turret.set(output);
        leaderMotorTurretSim.setInput(output*12);
        leaderMotorTurretSim.update(.02);
        
        leaderMotorFlywheelMech2d.setLength(
            leaderMotorVelocity.getValueAsDouble() / 100.0
        );
    }
    private void startSimThread() {
        turret.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        turret.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        angle.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {
            /* Calculate the time delta */
            final double currentTime = Utils.getCurrentTimeSeconds();
            final double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            final var leaderMotorSim = turret.getSimState();

            /* First set the supply voltage of all the devices */
            leaderMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

            /* Then calculate the new velocity of the simulated flywheel */
            leaderMotorTurretSim.setInputVoltage(leaderMotorSim.getMotorVoltage());
            leaderMotorTurretSim.update(deltaTime);

            /* Apply the new rotor velocity to the motors (before gear ratio) */
            leaderMotorSim.setRawRotorPosition(
                Radians.of(leaderMotorTurretSim.getAngularPositionRad() * kGearRatio)
            );
            leaderMotorSim.setRotorVelocity(
                RadiansPerSecond.of(leaderMotorTurretSim.getAngularVelocityRadPerSec() * kGearRatio)
            );
        });
        simNotifier.startPeriodic(kSimLoopPeriod);
    }


}
