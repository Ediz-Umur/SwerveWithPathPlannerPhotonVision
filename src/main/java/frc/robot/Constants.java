package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class Constants{

    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static boolean AutoBuilderConfigured = false;
    public static boolean MoveAndAlignEnabled = false;
    public static boolean xLock = false;
    public static final Shooter shooter = new Shooter();

    public static final double MinAmbiguity = 0.19;

    public class sDrivetrain{
        public static final double XYBase = 0.3; //TODO //Meters //The base std value for the translation
        public static final double RotBase = 0.5; //TODO //Degrees //The base std value for the rotation

        public static final double XYScale = 0.1; //TODO //Meters how much to increase the std dev by per meter of tag distance
        public static final double RotScale = 0.03; //TODO //Degrees how much to increase the std dev by per meter of tag distance

        public static final double XYMin = 0.5; //TODO //The minimum std value for the translation
        public static final double XYMax = 5; //TODO //The maximum std value for the translation
        public static final double RotMin = 0.5; //TODO //The minimum std value for the rotation
        public static final double RotMax = 5; //TODO //The maximum std value for the rotation
    }

    public class IntakeConstants{
        public final static int IntakeMotorID = 21;
        public final static double kA = 0.01;
        public final static double kS = 0.25;
        public final static double kV = 0.12;
        public final static double kP = 2;
        public final static double kI = 0.00;
        public final static double kD = 0.00;
        public final static double kMMAcceleration = 100;
        public final static double kMMJerk = kMMAcceleration * 10;
    }

    public class ShooterConstants{
        public final static int ShooterMotorID = 0; //TODO
        public final static int AngleMotorID = 0; //TODO

        public final static double kSShooter = 0.0; //TODO
        public final static double kVShooter = 0.0; //TODO
        public final static double kAShooter = 0.0; //TODO
        public final static double kPShooter = 0.0; //TODO
        public final static double kIShooter = 0.0; //TODO
        public final static double kDShooter = 0.0; //TODO

        public final static double mmAccelerationShooter = 0.0; //TODO
        public final static double mmJerkShooter = 0.0; //TODO

        public final static double kSAngle = 0.0; //TODO
        public final static double kVAngle = 0.0; //TODO
        public final static double kAAngle = 0.0; //TODO
        public final static double kGAngle = 0.0; //TODO
        public final static double kPAngle = 0.0; //TODO
        public final static double kIAngle = 0.0; //TODO
        public final static double kDAngle = 0.0; //TODO

        public final static double mmCruiseVelocityAngle = 0.0; //TODO
        public final static double mmkAAngle = 0.0; //TODO
        public final static double mmkVAngle = 0.0; //TODO

        public final static double shooterStatorCurrentLimit = 100000; //TODO
        public final static double angleStatorCurrentLimit = 100000; //TODO
    }

    public class AprilTagLimelight1Constants{
        public final static double limelightHeight = 0.203; //TODO
        public final static double limelightAngle = 30; //TODO
        public final static String limelightName = "limelight-shooter"; //TODO

        public static final double MaxEstimateSpeed = 180.0; //TODO
        public static final double MaxAmbiguity = 0.19; //TODO

        public static final double BaseXYStdDev = 0.3; //TODO
        public static final double BaseRotStdDev = 1.0; //TODO

        public static final double XYScaleFactor = 0.1; //TODO
        public static final double RotScaleFactor = 0.2; //TODO

        public static final double MultiTag2Plus = 0.5; //TODO
        public static final double MultiTag1 = 0.7; //TODO

        public static final double XYStdMin = 0.5; //TODO
        public static final double XYStdMax = 5.0; //TODO
        public static final double RotStdMin = 2.0; //TODO
        public static final double RotStdMax = 10.0; //TODO
    }
}