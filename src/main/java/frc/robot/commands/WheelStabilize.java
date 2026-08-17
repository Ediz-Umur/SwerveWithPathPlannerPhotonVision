package frc.robot.commands;

import java.lang.module.ModuleReader;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class WheelStabilize extends Command{


    public WheelStabilize(){

    }

    @Override
    public void initialize(){      
        Constants.drivetrain.getModule(0).apply(new ModuleRequest().withState(new SwerveModuleState(.1, Rotation2d.fromDegrees(315))));
        Constants.drivetrain.getModule(1).apply(new ModuleRequest().withState(new SwerveModuleState(.1, Rotation2d.fromDegrees(315))));
        Constants.drivetrain.getModule(2).apply(new ModuleRequest().withState(new SwerveModuleState(.1, Rotation2d.fromDegrees(315))));
        Constants.drivetrain.getModule(3).apply(new ModuleRequest().withState(new SwerveModuleState(.1, Rotation2d.fromDegrees(315))));
    
    }


    @Override
    public void execute(){

        

    }

    @Override
    public boolean isFinished(){
        return(false);
    }

    @Override
    public void end(boolean interrupted){
        Constants.drivetrain.getModule(0).getSteerMotor().setNeutralMode(NeutralModeValue.Brake, 1);
        Constants.drivetrain.getModule(0).getDriveMotor().setNeutralMode(NeutralModeValue.Brake, 1);
        Constants.drivetrain.getModule(1).getSteerMotor().setNeutralMode(NeutralModeValue.Brake, 1);
        Constants.drivetrain.getModule(1).getDriveMotor().setNeutralMode(NeutralModeValue.Brake, 1);
        Constants.drivetrain.getModule(2).getSteerMotor().setNeutralMode(NeutralModeValue.Brake,1);
        Constants.drivetrain.getModule(2).getDriveMotor().setNeutralMode(NeutralModeValue.Brake, 1);
        Constants.drivetrain.getModule(3).getSteerMotor().setNeutralMode(NeutralModeValue.Brake, 1);
        Constants.drivetrain.getModule(3).getDriveMotor().setNeutralMode(NeutralModeValue.Brake, 1);
    }
}
