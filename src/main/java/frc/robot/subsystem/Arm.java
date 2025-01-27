package frc.robot.subsystem;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.hardware.TalonFXMotor;

public class Arm extends SubsystemBase {
    TalonFXMotor armMotor = new TalonFXMotor(8);
    double feedforward = 0;

    public Arm() {
        this.armMotor.factoryDefault();
        this.armMotor.setInverted(false); //todo

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
        slot0Configs.kG = 0.1;
        slot0Configs.kS = 0.25;
        slot0Configs.kV = 0.05;
        slot0Configs.kA = 0.01;
        slot0Configs.kP = 0.1;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        this.armMotor.setSlot0Configs(slot0Configs);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.RotorToSensorRatio = 1;

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = 50;
        motionMagicConfigs.MotionMagicAcceleration = 100;
        motionMagicConfigs.MotionMagicJerk = 1000;
        this.armMotor.setMotionMagicConfigs(motionMagicConfigs);

        this.armMotor.shutdown();
        Timer.delay(3);
        this.armMotor.setPosition(0);
        this.armMotor.brake();
    }

    public void moveTo(double position) {
        this.armMotor.MotionMagicPosition(position);
    }

    public void brake() {
        this.armMotor.brake();
    }

    public double getPosition() {
        return this.armMotor.getPosition();
    }

    public void setFeedforward(double feedforward) {
        this.feedforward = feedforward;
    }
}

