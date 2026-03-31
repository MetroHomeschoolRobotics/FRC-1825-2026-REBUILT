package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
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

public class Turret extends SubsystemBase {
    
    private static CANcoder angle = new CANcoder(Constants.MotorIDs.turretCANcoderID,"*");
        private TalonFX turret = new TalonFX(Constants.MotorIDs.turretMotorID,"*");
        
        private DigitalInput beambreak = new DigitalInput(0);
        private static double robotAngle;
        private static double rotationCount = 0;
        private static int rotationCountInt = 0;
        private TalonFXConfiguration config = new TalonFXConfiguration();
       


        private boolean hasCorrectedNegative = false;
        private boolean hasCorrectedPositive = false;
        private static double setpoint=0;
                private static PIDController pid = new PIDController(Constants.PIDConstants.turretP,Constants.PIDConstants.turretI, Constants.PIDConstants.turretD);
        
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
                    if(Utils.isSimulation()){
                    startSimThread();
                }
                    setConfigs();
                    turret.getConfigurator().apply(config);
                }
                private void setConfigs(){
                config.CurrentLimits.StatorCurrentLimit = 40;
                config.MotorOutput.Inverted =InvertedValue.CounterClockwise_Positive;
                config.SoftwareLimitSwitch.ForwardSoftLimitEnable=true;
                config.SoftwareLimitSwitch.ForwardSoftLimitThreshold=Constants.Setpoints.turretForwardSoftLimit;
                config.SoftwareLimitSwitch.ReverseSoftLimitEnable=true;
                config.SoftwareLimitSwitch.ReverseSoftLimitThreshold=Constants.Setpoints.turretReverseSoftLimit;
        
            }
                public static void turretSetSetpoint(double _setpoint){
                    setpoint = _setpoint;
            
            
            
        }
        public static void setRobotAngle(double angle){
            
            robotAngle = angle-45;
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
            output =rotationCount*9;
            return output+robotAngle;

        }
       public void set(double num){
        turret .set(num);
        
       }
        public void setPID(double angle){
            pid.setSetpoint(angle);
        }
        public static double getAbsoluteAngle(){
            double output =rotationCount*9;
           
            // if(output>180){
            //     output-=360;
            //     // rotationCount+=0.5;
            // }else if(output<-180){
                
            //     output+=360;
            //     // rotationCount-=0.5;
            // }
            return output;
    }
    public void incrementTurretAngle(double input){
        setpoint+=input;
    }
    /**this treats 0 as facing the intake, the shooter starts facing 125 (125 degrees CW) */
    public void fixSetpoint(){
        
        String sameCorrectionFlag ="";
        if(setpoint>=171){
                setpoint-=348;
                sameCorrectionFlag="125";
                hasCorrectedPositive = true;
            }else if(setpoint<-177){
                setpoint+=348;
                sameCorrectionFlag="-230";
                hasCorrectedNegative=true;
            }
        // if(getAbsoluteAngle()>=171){
            
        //         setpoint-=355.5;
        //         hasCorrectedPositive = true;
                
        //     }else if(getAbsoluteAngle()<-184.5){
               
        //         setpoint+=355.5;
        //         hasCorrectedNegative=true;
        //     }
        //     if(getAbsoluteAngle()<20&&getAbsoluteAngle()>-20){
        //         hasCorrectedNegative=false;
        //         hasCorrectedPositive=false;
        //     }
        pid.setSetpoint(setpoint);
    }
    public void periodic(){
        //DOES THIS WORK, IDK
        if(!beambreak.get()){
            turret.setPosition(Constants.Setpoints.turretForwardSoftLimit);
        }
       rotationCount=turret.getPosition().getValueAsDouble();
       
       fixSetpoint();
      
        double output = pid.calculate(getAbsoluteAngle());

        output = MathUtil.clamp(output, -.3, .3);
         if(turret.getPosition().getValueAsDouble()>=Constants.Setpoints.turretForwardSoftLimit&&output>0){
            output=0;
        }else if(turret.getPosition().getValueAsDouble()<Constants.Setpoints.turretReverseSoftLimit&&output<0){
            output=0;
        }
        //turret.set(output);
      
        SmartDashboard.putBoolean("has corrected negative", hasCorrectedNegative);
        SmartDashboard.putBoolean("has corrected positive", hasCorrectedPositive);
        SmartDashboard.putNumber("turret motor encoder ", turret.getPosition().getValueAsDouble());
        //0.37 to -9.63 90ish degrees to the right
        //-14.9 rightmost limit
        //
        SmartDashboard.putNumber("setpoint minus angle", pid.getSetpoint()-robotAngle);
        SmartDashboard.putNumber("absolute angle", getAbsoluteAngle());
        SmartDashboard.putNumber("rotation count", rotationCountInt);
        SmartDashboard.putNumber("geared angle", getGearedAngle());
        SmartDashboard.putNumber("setpoint turret", pid.getSetpoint());
        SmartDashboard.putBoolean("turret beambreak", !beambreak.get());

    }
    public void simulationPeriodic(){
        angleSim.setVelocity(leaderMotorTurretSim.getAngularVelocityRPM()/60);
        angleSim.addPosition((leaderMotorTurretSim.getAngularVelocityRPM()/60)/50);
        fixSetpoint();
        double output = pid.calculate(getAbsoluteAngle());
        if(turret.getPosition().getValueAsDouble()>=Constants.Setpoints.turretForwardSoftLimit&&output>0){
            output=0;
        }else if(turret.getPosition().getValueAsDouble()<Constants.Setpoints.turretReverseSoftLimit&&output<0){
            output=0;
        }
        output = MathUtil.clamp(output, -.15, .15);
        //turret.set(output);
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
