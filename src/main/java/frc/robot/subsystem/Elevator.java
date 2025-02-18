package frc.robot.subsystem;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
        this.elevatorMotor.MotionMagic(position.getSensorPosition(), this.getFeedforward(position));
    }

    public void moveTo(Distance height) {
        this.moveTo(new Position(height));
    }

    public void moveTo(double heightInMeters) {
        this.moveTo(new Position(heightInMeters));
    }

    private double getFeedforward(Position position) {
        return this.getFeedforward(position.sensorPosition);
    }

    private double getFeedforward(Angle sensorPosition) {
        //TODO: measure feedforward
        return 0;
    }

    public void brake() {
        this.elevatorMotor.brake();
    }

    public Position getPosition() {
        return new Position(this.elevatorMotor.getPosition());
    }

    public Command getMoveCommand(Position position) {
        return Commands.runOnce(() -> this.moveTo(position));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Elevator");
        builder.setActuator(true);
        builder.setSafeState(this::brake);
        builder.addDoubleProperty("Position", () -> this.getPosition().getHeight().in(Units.Meters), this::moveTo);
        builder.addBooleanProperty("Limit Switch", this.limitSwitch::get, null);
        SmartDashboard.putData("Move to Bottom", this.getMoveCommand(Level.BOTTOM.position));
        SmartDashboard.putData("Move to L1", this.getMoveCommand(Level.L1.position));
        SmartDashboard.putData("Move to L2", this.getMoveCommand(Level.L2.position));
        SmartDashboard.putData("Move to L3", this.getMoveCommand(Level.L3.position));
        SmartDashboard.putData("Move to L4", this.getMoveCommand(Level.L4.position));
        SmartDashboard.putData("Reset", Commands.runOnce(this::reset).ignoringDisable(true));
    }

    public void reset() {
        this.elevatorMotor.resetPosition();
    }

    enum Level {
        BOTTOM(0), L1(0.5), L2(1), L3(1.5), L4(1.8);

        private final Position position;

        Level(double heightInMeters) {
            this.position = new Position(heightInMeters);
        }
    }

    public static class Position {
        private static final double SENSOR_ROTATION_TO_METER_RATIO = 1; //TODO

        private final Angle sensorPosition;

        public Position(Angle sensorPosition) {
            this.sensorPosition = sensorPosition;
        }

        public Position(Distance height) {
            this.sensorPosition = Units.Rotations.of(height.in(Units.Meters) / SENSOR_ROTATION_TO_METER_RATIO);
        }

        public Position(double heightInMeters) {
            this.sensorPosition = Units.Rotations.of(heightInMeters / SENSOR_ROTATION_TO_METER_RATIO);
        }

        private Angle getSensorPosition() {
            return sensorPosition;
        }

        public Distance getHeight() {
            return Units.Meters.of(getSensorPosition().in(Units.Rotations) * SENSOR_ROTATION_TO_METER_RATIO);
        }
    }
}

