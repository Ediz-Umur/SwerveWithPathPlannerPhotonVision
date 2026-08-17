package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final TalonFX Intakemotor;
    private final TalonFXConfiguration intakemotorConfiguration;
    private final MotionMagicVelocityTorqueCurrentFOC intakeCurrentFOC;
    public Intake(){
        Intakemotor = new TalonFX(Constants.IntakeConstants.IntakeMotorID, "Civ Civ");
        intakemotorConfiguration = new TalonFXConfiguration();
        intakemotorConfiguration.Slot0.kA = Constants.IntakeConstants.kA;
        intakemotorConfiguration.Slot0.kS = Constants.IntakeConstants.kS;
        intakemotorConfiguration.Slot0.kV = Constants.IntakeConstants.kV;
        intakemotorConfiguration.Slot0.kP = Constants.IntakeConstants.kP;
        intakemotorConfiguration.Slot0.kI = Constants.IntakeConstants.kI;
        intakemotorConfiguration.Slot0.kD = Constants.IntakeConstants.kD;
        intakemotorConfiguration.Feedback.SensorToMechanismRatio = 2.5;

        final MotionMagicConfigs motionMagicConfigs = intakemotorConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = Constants.IntakeConstants.kMMAcceleration;
        motionMagicConfigs.MotionMagicJerk = Constants.IntakeConstants.kMMJerk;

        Intakemotor.getConfigurator().apply(intakemotorConfiguration);
        
        intakeCurrentFOC = new MotionMagicVelocityTorqueCurrentFOC(25);

    }
    public void runIntake(){
        Intakemotor.setControl(intakeCurrentFOC.withVelocity(25).withSlot(0));
;
    }
    public void StopIntake(){
        Intakemotor.setControl(intakeCurrentFOC.withVelocity(0).withSlot(0));


    }
}
