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
    TalonSRXMotor elevatorLeftLeader = new TalonSRXMotor(5);
    TalonSRXMotor elevatorLeftFollower = new TalonSRXMotor(6);
    TalonSRXMotor elevatorRightFollower1 = new TalonSRXMotor(7);
    TalonSRXMotor elevatorRightFollower2 = new TalonSRXMotor(8);
    DigitalInput limitSwitch = new DigitalInput(0);
    Position targetPosition;

    public Elevator() {
        this.elevatorLeftLeader.factoryDefault();
        this.elevatorLeftFollower.factoryDefault();
        this.elevatorRightFollower1.factoryDefault();
        this.elevatorRightFollower2.factoryDefault();
        this.elevatorLeftLeader.setMotionMagicConfig(0.1, 0, 0, 0.01, Units.RadiansPerSecond.of(Math.PI),
                Units.RadiansPerSecondPerSecond.of(Math.PI), 3);

        this.elevatorLeftLeader.setInverted(false);
        this.elevatorLeftLeader.setPhase(false);

        this.elevatorLeftFollower.follow(elevatorLeftLeader);
        this.elevatorRightFollower1.follow(elevatorLeftFollower, true);
        this.elevatorRightFollower2.follow(elevatorLeftLeader, true);

        this.elevatorLeftLeader.shutdown();
        Timer.delay(3);
        this.elevatorLeftLeader.brake();
        this.elevatorLeftLeader.resetPosition();
        this.targetPosition = null;
    }

    @Override
    public void periodic() {
        if (this.atBottom()) {
            this.elevatorLeftLeader.resetPosition();
        }
        if (this.targetPosition != null) {
            double feedforward = this.getFeedforward(this.targetPosition);
            this.elevatorLeftLeader.MotionMagic(this.targetPosition.getSensorPosition(), feedforward);
        } else {
            this.elevatorLeftLeader.shutdown();
        }
    }

    public void moveTo(Position position) {
        this.targetPosition = position;
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
        this.elevatorLeftLeader.brake();
        this.targetPosition = this.getPosition();
    }

    public void shutdown() {
        this.elevatorLeftLeader.shutdown();
        this.targetPosition = null;
    }

    public Position getPosition() {
        return new Position(this.elevatorLeftLeader.getPosition());
    }

    public Command getMoveCommand(Position position) {
        return runOnce(() -> this.moveTo(position));
    }

    public boolean atBottom() {
        return !this.limitSwitch.get();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Elevator");
        builder.setActuator(true);
        builder.setSafeState(this::brake);
        builder.addDoubleProperty("Position", () -> this.getPosition().getHeight().in(Units.Meters), this::moveTo);
        builder.addBooleanProperty("At Bottom", this::atBottom, null);
        SmartDashboard.putData("Move to Bottom", this.getMoveCommand(Level.BOTTOM.position));
        SmartDashboard.putData("Move to L1", this.getMoveCommand(Level.L1.position));
        SmartDashboard.putData("Move to L2", this.getMoveCommand(Level.L2.position));
        SmartDashboard.putData("Move to L3", this.getMoveCommand(Level.L3.position));
        SmartDashboard.putData("Move to L4", this.getMoveCommand(Level.L4.position));
        SmartDashboard.putData("Reset", Commands.runOnce(this::reset).ignoringDisable(true));
        SmartDashboard.putData("run", runOnce(() -> {
            this.elevatorLeftLeader.power(0.2);
        }));
    }

    public void reset() {
        this.elevatorLeftLeader.resetPosition();
        this.targetPosition = null;
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

