package frc.robot.subsystem;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
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
        this.elevatorMotor.setMotionMagicConfig(0.1, 0, 0, 0.01, Units.RadiansPerSecond.of(Math.PI * 10),
                Units.RadiansPerSecondPerSecond.of(Math.PI * 10), 3);

        this.elevatorMotor.shutdown();
        Timer.delay(3);
        this.elevatorMotor.brake();

        this.elevatorMotor.resetPosition();
        this.elevatorMotor.setInverted(false); //TODO
        this.elevatorMotorFollower.follow(this.elevatorMotor, false);
        this.elevatorMotor.setPhase(false); //TODO
    }

    @Override
    public void periodic() {
        if (this.limitSwitch.get()) {
            this.elevatorMotor.resetPosition();
        }
    }

    public void moveTo(Position position) {
        this.elevatorMotor.MotionMagic(position.getMotorPosition(), this.getFeedforward());
    }

    private double getFeedforward() {
        return this.getFeedforward(this.elevatorMotor.getPosition());
    }

    private double getFeedforward(Angle motorPosition) {
        //TODO: measure feedforward
        return 0;
    }

    public void brake() {
        this.elevatorMotor.brake();
    }

    public Position getPosition() {
        return new Position(this.elevatorMotor.getPosition());
    }

    public static class Position {
        private final Angle motorPosition;

        public Position(Angle motorPosition) {
            this.motorPosition = motorPosition;
        }

        private Angle getMotorPosition() {
            return motorPosition;
        }
    }
}

