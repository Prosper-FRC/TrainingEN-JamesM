package frc.robot.subsystems;

public class PivotConstants {
    public static final int rightPivotID = 41;
    public static final int leftPivotID = 42;
    public static final int pivotEncoderID = 0;
    public static final double pivotGearRatio = 96.9/1;

    public static final double pivotKp = 0.002;
    public static final double pivotKi = 0;
    public static final double pivotKd = 0;
    public static final double pivotKG = 0.0003;

    public static final double offset = 58.669+6;

    public static final class Setpoints {
		public static final double subwooferAngle = 60.0;
		public static final double passingAngle = 65.0;
		public static final double podiumAngle = 0;
		public static final double ampAngle = 150.0;
		public static final double overheadSubwooferAngle = 165.0;

		public static final double idleAngle = 92;
		public static final double zero = 0;
}
}
