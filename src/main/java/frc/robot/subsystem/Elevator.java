package frc.robot.subsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.hardware.TalonSRXMotor;

public class Elevator extends SubsystemBase {
    TalonSRXMotor elevatorMotor = new TalonSRXMotor(7);
    double feedforward = 0;

    public Elevator() {
        this.elevatorMotor.factoryDefault();
        this.elevatorMotor.setInverted(false); //todo
        this.elevatorMotor.setMotionMagicConfig(0.1, 0, 0, 0.01, Math.PI * 10, Math.PI * 10, 3);

        this.elevatorMotor.shutdown();
        Timer.delay(3);
        this.elevatorMotor.setPosition(0);
        this.elevatorMotor.brake();
    }

    public void moveTo(double position) {
        this.elevatorMotor.MotionMagic(position, feedforward);
    }

    public void brake() {
        this.elevatorMotor.brake();
    }

    public double getPosition() {
        return this.elevatorMotor.getPosition();
    }

    public void setFeedforward(double feedforward) {
        this.feedforward = feedforward;
    }
}

