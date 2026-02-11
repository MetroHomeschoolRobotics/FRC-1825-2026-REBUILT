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
        public static final int indexerID = 18;
        public static final int beltID = 19;
        public static final int turretMotorID = 20;
        public static final int turretCANID=22;
        public static final int hoodID1=23;
        public static final int hoodID2=24;
    }
    public static class MathConstants{
        public static final double hoodRotationsPerDegree=1.0;
        public static final double defaultHoodAngle = 58.337;

        public static final double shooterWheelRadius = Units.inchesToMeters(4);
        public static final double shooterWheelCircumference = 2*Math.PI*shooterWheelRadius;
    }
    public static class FieldSetpoints{
        // uncomment when the april tag layout is released
        public static final Pose2d blueHubPose = new Pose2d();
        public static final Pose2d redHubPose = new Pose2d();
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
    public static class CameraPositions {
    public static final Transform3d frontLeftTranslation = new Transform3d(
        Units.inchesToMeters(10.9),
        Units.inchesToMeters(10.8), // 11.29
        Units.inchesToMeters(9.321819),
        new Rotation3d(
            Units.degreesToRadians(0),
            Units.degreesToRadians(30),
            Units.degreesToRadians(0)));

    public static final Transform3d frontRightTranslation = new Transform3d(
        Units.inchesToMeters(-10.9),
        Units.inchesToMeters(10.8), // -11.29
        Units.inchesToMeters(9.2),
        new Rotation3d(
            Units.degreesToRadians(0),
            Units.degreesToRadians(0),
            Units.degreesToRadians(0)));
                                                              
  }
}
