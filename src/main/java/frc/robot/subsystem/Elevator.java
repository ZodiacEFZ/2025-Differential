package frc.robot.subsystem;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.hardware.TalonSRXMotor;

public class Elevator extends SubsystemBase {
    TalonSRXMotor elevatorMotor = new TalonSRXMotor(11);
    TalonSRXMotor elevatorMotorFollower = new TalonSRXMotor(12);
    DigitalInput limitSwitch = new DigitalInput(0);

    public Elevator() {
        this.elevatorMotor.factoryDefault();
        this.elevatorMotorFollower.factoryDefault();
        this.elevatorMotor.setMotionMagicConfig(0.1, 0, 0, 0.01, Math.PI * 10, Math.PI * 10, 3);

        this.elevatorMotor.shutdown();
        Timer.delay(3);
        this.elevatorMotor.brake();

        this.elevatorMotor.resetPosition();
        this.elevatorMotor.setInverted(false); //TODO
        this.elevatorMotorFollower.follow(this.elevatorMotor, false);
        this.elevatorMotor.setPhase(false); //TODO
    }

    public void moveTo(double position) {
        this.elevatorMotor.MotionMagic(position, this.getFeedforward());
    }

    private double getFeedforward() {
        return this.getFeedforward(this.elevatorMotor.getPosition());
    }

    private double getFeedforward(double position) {
        if (position <= 256) {
            return 0;
        }
        if (position < 4096) {
            return 0.1;
        }
        return 0;
    }

    public void brake() {
        this.elevatorMotor.brake();
    }

    public double getPosition() {
        return this.elevatorMotor.getPosition();
    }
}

