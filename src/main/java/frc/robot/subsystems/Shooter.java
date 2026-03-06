package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;


public class Shooter extends SubsystemBase{
    private PIDController pid = new PIDController(Constants.PIDConstants.shooterP,Constants.PIDConstants.shooterI,Constants.PIDConstants.shooterD);
    
    private double desiredVelocity = 0;
    private InterpolatingDoubleTreeMap interpolation = new InterpolatingDoubleTreeMap();
    private TalonFX shooter1 = new TalonFX(Constants.MotorIDs.shooterMotorID1);
    private TalonFX shooter2 = new TalonFX(Constants.MotorIDs.shooterMotorID2);
    
    private final Mechanism2d leaderMotorMech2d = new Mechanism2d(2, 2);
    private final MechanismLigament2d leaderMotorFlywheelMech2d = leaderMotorMech2d.getRoot("Flywheel Root leaderMotor", 1, 1)
        .append(new MechanismLigament2d("Flywheel leaderMotor", 1, 0));

     private final StatusSignal<AngularVelocity> leaderMotorVelocity = shooter1.getVelocity(false);
    private static final double kGearRatio = 1;
        private final DCMotor leaderMotorDCMotors = DCMotor.getKrakenX60Foc(2);
    private final LinearSystem<N2, N1, N2> leaderMotorFlywheelSystem = LinearSystemId.createDCMotorSystem(leaderMotorDCMotors, 0.004, kGearRatio);
    private final DCMotorSim leaderMotorFlywheelSim = new DCMotorSim(leaderMotorFlywheelSystem, leaderMotorDCMotors);

    private static final double kSimLoopPeriod = 0.002; // 2 ms
    private Notifier simNotifier = null;
    private double lastSimTime = 0.0;
    public Shooter(){
        pid.setTolerance(50);
         double[] inputs = Constants.InterpolationData.inputs;
         double[] outputs = Constants.InterpolationData.outputs;
        for (int i = 0 ; i<inputs.length;i++) {
            interpolation.put(inputs[i], outputs[i]);
        } 
        startSimThread();
    }
    public double getInterpolatedRPM(double distToHub){
        double targetRPM = interpolation.get(distToHub);
        return targetRPM;
    }

    public void setSpeed(double speed){
        shooter1.set(speed);
        shooter2.set(-speed);
    }
    public boolean atSetpoint(){
        return pid.atSetpoint();
    }
    public void incrementRPM(double joystickInput){
        desiredVelocity = desiredVelocity + (5*-joystickInput);
        pid.setSetpoint(desiredVelocity);
    }
    public void setRPM(double RPM){
        desiredVelocity = RPM;
        pid.setSetpoint(desiredVelocity);
    }

    public double getRPM(){
       
        return shooter1.getVelocity().getValueAsDouble()*60;
    }
    /**Meters per second **/
    public double estimatedFuelVelocity(){
        
        return ((getRPM()*Constants.MathConstants.shooterWheelCircumference)/60)/2;
    }
   
    public void periodic(){
        double output = pid.calculate(getRPM());
        setSpeed(output);
        SmartDashboard.putNumber("ShooterRPM", getRPM());
        SmartDashboard.putNumber("Fuel Velocity", estimatedFuelVelocity());
        
        
    }
    public void simulationPeriodic(){
        double output = pid.calculate(getRPM());
        setSpeed(output);
        leaderMotorFlywheelSim.setInput(output*12);
        leaderMotorFlywheelSim.update(.02);
        
        leaderMotorFlywheelMech2d.setLength(
            leaderMotorVelocity.getValueAsDouble() / 100.0
        );
    }
    private void startSimThread() {
        shooter1.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        shooter1.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);

        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {
            /* Calculate the time delta */
            final double currentTime = Utils.getCurrentTimeSeconds();
            final double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            final var leaderMotorSim = shooter1.getSimState();

            /* First set the supply voltage of all the devices */
            leaderMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

            /* Then calculate the new velocity of the simulated flywheel */
            leaderMotorFlywheelSim.setInputVoltage(leaderMotorSim.getMotorVoltage());
            leaderMotorFlywheelSim.update(deltaTime);

            /* Apply the new rotor velocity to the motors (before gear ratio) */
            leaderMotorSim.setRawRotorPosition(
                Radians.of(leaderMotorFlywheelSim.getAngularPositionRad() * kGearRatio)
            );
            leaderMotorSim.setRotorVelocity(
                RadiansPerSecond.of(leaderMotorFlywheelSim.getAngularVelocityRadPerSec() * kGearRatio)
            );
        });
        simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
