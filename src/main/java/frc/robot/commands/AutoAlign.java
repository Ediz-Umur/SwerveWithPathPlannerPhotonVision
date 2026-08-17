package frc.robot.commands;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;

public class AutoAlign extends Command{

    //Sets up the variables
    private Translation2d TargetTranslation;
    private Translation2d CurrentTranslation;
    private Translation2d TargetVector;
    private double Rotation;
    private double Goal;
    private PIDController PID = new PIDController(0.15, 0, 0); //TODO //NOT SURE WHETHER THEY WORK OR NOT //The PID gains for the hub alignment
    private double Ticker;
    private Timer Timer = new Timer();

    //The tolerances for the various end conditions, mainly change this and the PID gains to tune the command
    private double TickerTolerance = 30; //TODO
    private double DegreeTolerance = 1.0; //TODO
    private double TimerTolerance = 99999999; //TODO

    public AutoAlign(){
    }

    @Override
    public void initialize(){
        //Resets and starts the timer
        Timer.reset();
        Timer.start();
        PID.enableContinuousInput(-180, 180);

        //Sets the target translation
        if (true){
            TargetTranslation = new Translation2d(4.625594, 4.034536);
        } else {
            TargetTranslation = new Translation2d(11.915394, 4.034536);
        }
    }

    @Override 
    public void execute(){
        
        //Gets the target rotation to go to
        CurrentTranslation = Constants.drivetrain.getState().Pose.getTranslation();
        TargetVector = CurrentTranslation.minus(TargetTranslation);
        Goal = TargetVector.getAngle().getDegrees();

        Rotation = Constants.drivetrain.getState().Pose.getRotation().getDegrees(); //Calculates the current postion of the robot

        SwerveRequest Drive = new SwerveRequest.FieldCentric().withRotationalRate(PID.calculate(Rotation + 180, Goal)); //Creates a new swerve request to move the drivetrain the the desired angle
        CommandScheduler.getInstance().schedule(Constants.drivetrain.applyRequest(() -> Drive).until(() -> Ticker > TickerTolerance || Timer.get() > TimerTolerance)); //Sends the command to move the drivetrain

        double Error = Math.abs(Goal - (Rotation + 180));
        if (Error < DegreeTolerance){ //If the error between the current and desired angle is within tolerance, updates the ticker and sets the light
            Ticker += 1;
        } else if (Error < DegreeTolerance * 1.5){ //If the error is close to the desired angle, updates the ticker slower and sets the light
            Ticker += 0.5;
        } else { //If the error goes too far, resets the ticker and sets the light
            Ticker = 0;
        }

    }

    @Override
    public boolean isFinished(){
        //If the ticker gets too long, there is no visible apriltag, or too much time passed, stops the command
        return(Ticker > TickerTolerance ||  Timer.get() > TimerTolerance); //TODO
    }

    @Override
    public void end(boolean Interrupted){
        //Sets the light
        PID.close();
    }
}