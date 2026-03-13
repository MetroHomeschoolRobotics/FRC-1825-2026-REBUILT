package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static class MotorIDs{
        public static final int shooterMotorID1 = 14;
        public static final int shooterMotorID2 = 15;

        public static final int intakeID = 16;
        public static final int intakeRetractorID = 17;
        public static final int intakeCANcoderID = 26;

        public static final int indexerID = 18;
        public static final int beltID = 19;

        public static final int turretMotorID = 20;
        public static final int turretCANcoderID=22;

        public static final int hoodID1=0;
        public static final int hoodID2=1;//PWM channels

        public static final int CANDiId = 25;
    }
   
    public static class MathConstants{
        public static final double hoodRotationsPerDegree=1.0;

        public static final double shooterWheelRadius = Units.inchesToMeters(4.0/2.0); //diameter/2
        public static final double shooterWheelCircumference = 2*Math.PI*shooterWheelRadius;
    }
    public static class FieldSetpoints{
      
        public static final Pose2d blueHubPose = new Pose2d(Units.inchesToMeters(181.56),Units.inchesToMeters(158), null);
        public static final Pose2d redHubPose = new Pose2d(Units.inchesToMeters(469.62),Units.inchesToMeters(158), null);
        
         public static final double blueAllianceZoneX= Units.inchesToMeters(144.19);
        public static final double redAllianceZoneX = Units.inchesToMeters(464.63);
        public static final double upperYValue = Units.inchesToMeters(181.5+29);
        public static final double lowerYValue = Units.inchesToMeters(181.5-29);
        
        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    }
    public static class TimerConstants{
        //
        public static final double transitionStart = 140;
        public static final double shift1Start = 130;
        public static final double shift2Start = 105;
        public static final double shift3Start = 80;
        public static final double shift4Start = 55;
        public static final double endgameStart = 30;
        public static final double[] shiftTimes= {
            transitionStart,
            shift1Start,
            shift2Start,
            shift3Start,
            shift4Start,
            endgameStart
        };
    
 
    }
    public static class Setpoints{
        public static final double defaultHoodAngle = 58.337;
        public static final double passingHoodAngle = 45;
        public static final double retractorDeploySpeed = -0.3;
        public static final double retractorRetractSpeed = 0.4;
        public static final double indexerSpeed = 0.3;
        public static final double beltSpeed = 0.3;
        public static final double intakeSpeed = 0.5;
    }   
    public static class InterpolationData{
        //Fake data made using a trajectory calculator
        public static final double[] inputs = {
            Units.inchesToMeters(90.62),//0
            Units.inchesToMeters(120.62),//1
            Units.inchesToMeters(140),//2
            Units.inchesToMeters(159.59),//3
            Units.inchesToMeters(213.1)//4
        };
        public static final double[] outputs = {
            2475,//0
            2635,//1
            2750,//2
            2910,//3
            3265//4
        };
        //TO/DO populate this
    }
    public static class PIDConstants{
        public static final double hoodP =0.01;
        public static final double hoodI =0;
        public static final double hoodD =0;

        public static final double shooterP = 0.0019;
        public static final double shooterI = 0.0012;
        public static final double shooterD= 0.0014;
        
        public static final double turretP = 0.0009;
        public static final double turretI = 0.000;
        public static final double turretD = 0;
    }
    public static class CameraPositions {
    public static final Transform3d frontLeftTranslation = new Transform3d(
        Units.inchesToMeters(10.9),
        Units.inchesToMeters(-10.8), // 11.29
        Units.inchesToMeters(9.321819),
        new Rotation3d(
            Units.degreesToRadians(0),
            Units.degreesToRadians(-30),
            Units.degreesToRadians(90)));

//     public static final Transform3d frontRightTranslation = new Transform3d(
//         Units.inchesToMeters(-10.9),
//         Units.inchesToMeters(10.8), // -11.29
//         Units.inchesToMeters(9.2),
//         new Rotation3d(
//             Units.degreesToRadians(0),
//             Units.degreesToRadians(0),
//             Units.degreesToRadians(0)));
                                                              
   }
 }
