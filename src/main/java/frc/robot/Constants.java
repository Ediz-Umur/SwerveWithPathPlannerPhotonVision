package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
//import frc.robot.subsystems.Shooter;

public class Constants{

    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static boolean AutoBuilderConfigured = false;
    public static boolean MoveAndAlignEnabled = false;
    public static boolean xLock = false;
    //public static final Shooter shooter = new Shooter();

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

    public class VisionCameraKargaConstants{
        public final static String Name = "Mavi";
        public static final Transform3d RobotToCamera = new Transform3d(new Translation3d(0.355,0,0.132), new Rotation3d(-Units.degreesToRadians(30),0,0));
    }

    public class VisionCameraTavukConstants{
        public final static String Name = "Yesil";
        public static final Transform3d RobotToCamera = new Transform3d(new Translation3d(0, -0.355, 0.132), new Rotation3d(0,Units.degreesToRadians(30),Units.degreesToRadians(90)));
    }

    public class VisionCameraSiyahConstants{
        public final static String Name = "Siyah";
        public static final Transform3d RobotToCamera = new Transform3d(new Translation3d(0,0.355,0.132), new Rotation3d(Units.degreesToRadians(30),0,-Units.degreesToRadians(90)));
    }
    
    public class VisionStdConstants{
        public final static Matrix<N3, N1> stdSingleTag = VecBuilder.fill(1.1, 1.1, 1.5);
        public final static Matrix<N3, N1> stdMultitag = VecBuilder.fill(0.7, 0.7, 0.9);

        public final static double AvgMaxDistance = 4; //How far the avgDist has to be for a 1 tag pose before the pose is trashed
        public final static double StdPenalty = 30; //How much the Std dev is increased for 1 tag poses (LOWER MEANS MORE PENALTY)

        public final static double MaxAmbiguity = 0.2; //THe maximum ambiguity in a target before the pose estimate is trashed
    }
}