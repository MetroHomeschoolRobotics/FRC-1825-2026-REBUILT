package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

public final class Constants {
    public static class MotorIDs{
        public static final int shooterMotorID1 = 14;
        public static final int shooterMotorID2 = 15;
        public static final int intakeID = 16;
        public static final int indexerID = 17;
        public static final int beltID = 18;
        public static final int turretMotorID = 19;
        public static final int turretCANID=20;
        public static final int hoodID1=21;
        public static final int hoodID2=22;
    }
    public static class MathConstants{
        public static final double hoodRotationsPerDegree=1.0;
        public static final double defaultHoodAngle = 58.337;
    }
    public static class FieldSetpoints{
        // uncomment when the april tag layout is released
        //public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField();
    }
}
