// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.invoke.ConstantCallSite;
import java.lang.reflect.Array;
import java.rmi.StubNotFoundException;
import java.util.ArrayList;

import javax.sound.sampled.SourceDataLine;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AprilTagLimelight1Constants;
import frc.robot.subsystems.Shooter;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    private Field2d field = new Field2d();

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        LimelightHelpers.SetIMUMode(AprilTagLimelight1Constants.limelightName, 1);
        SmartDashboard.putData("Field", field);
    }

    @Override
    public void robotPeriodic() {
        field.setRobotPose(Constants.drivetrain.getState().Pose);
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        /*
         * This example of adding Limelight is very simple and may not be sufficient for on-field use.
         * Users typically need to provide a standard deviation that scales with the distance to target
         * and changes with number of tags available.
         *
         * This example is sufficient to show that vision integration is possible, though exact implementation
         * of how to use vision should be tuned per-robot and to the team's specification.
         */
            var driveState = Constants.drivetrain.getState();
            double headingDeg = driveState.Pose.getRotation().getDegrees();
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
            double omegaDps = Units.radiansToDegrees(Units.rotationsPerMinuteToRadiansPerSecond(omegaRps*60));

            SmartDashboard.putNumber("Yaw", headingDeg);
            LimelightHelpers.SetRobotOrientation(Constants.AprilTagLimelight1Constants.limelightName, headingDeg, omegaDps, 0, 0, 0, 0);
            var llMeasurementShooter = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.AprilTagLimelight1Constants.limelightName);
            if (!Constants.drivetrain.RejectMeasurement(llMeasurementShooter)) {
                Constants.drivetrain.addVisionMeasurement(llMeasurementShooter.pose, 
                                                         llMeasurementShooter.timestampSeconds - (llMeasurementShooter.latency + LimelightHelpers.getLatency_Capture(Constants.AprilTagLimelight1Constants.limelightName)) / 100, 
                                                         VecBuilder.fill(0.7, 0.7, 999999));
            }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        //DELETE THIS WHEN THE AUTONOMOUS COMMAND IS DONE
        //TODO
        Constants.drivetrain.resetPose(new Pose2d(new Translation2d(3.990, 7.385), new Rotation2d(0)));
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
