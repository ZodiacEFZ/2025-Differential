package frc.robot.subsystem;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.hardware.TalonSRXMotor;
import frc.libzodiac.ui.Elastic;

import java.util.Objects;

public class Elevator extends SubsystemBase {
    // The elevator motors.
    TalonSRXMotor elevatorLeftLeader = new TalonSRXMotor(5);
    TalonSRXMotor elevatorLeftFollower = new TalonSRXMotor(6);

    // The bottom limit switch.
    DigitalInput limitSwitch = new DigitalInput(0);

    // The target position of the elevator.
    Position targetPosition;
    // Whether the elevator has reset to zero.
    boolean hasResetToZero = false;
    boolean isTryingGoDown = true;

    public Elevator() {
        // Configure the elevator motors.
        this.elevatorLeftLeader.factoryDefault();
        this.elevatorLeftFollower.factoryDefault();
        this.elevatorLeftLeader.setMotionMagicConfig(0.1, 0.005, 1, 0.125, Units.RadiansPerSecond.of(16 * Math.PI),
                Units.RadiansPerSecondPerSecond.of(16 * Math.PI), 3);
        this.elevatorLeftLeader.setMaxIntegralAccum(27500);
        this.elevatorLeftLeader.setPeakOutput(0.5);
        this.elevatorLeftLeader.allowableError(200);

        this.elevatorLeftLeader.setInverted(false);
        this.elevatorLeftLeader.setPhase(false);

        this.elevatorLeftFollower.follow(elevatorLeftLeader);

        this.elevatorLeftLeader.setBrakeWhenNeutral(true);

        // Reset the elevator position.
        this.elevatorLeftLeader.resetPosition();
        this.targetPosition = null;
    }

    /**
     * Get the target position that is within the limits.
     *
     * @param target the original target position
     * @return the target position that is within the limits
     */
    private static Position getMovablePosition(Position target) {
        if (target.getSensorPosition().in(Units.Radians) > Level.L4.position.getSensorPosition().in(Units.Radians)) {
            return Level.L4.position;
        }
        if (target.getSensorPosition().in(Units.Radians) < 0) {
            return new Position(Units.Radians.of(0));
        }
        return target;
    }

    @Override
    public void periodic() {
        if (this.getAtBottomState()) {
            if (this.isTryingGoDown) {
                this.elevatorLeftLeader.brake();
            }
            if (this.hasResetToZero) {
                this.elevatorLeftLeader.setBrakeWhenNeutral(true);
            }
            this.elevatorLeftLeader.resetPosition();
            this.hasResetToZero = true;
            this.isTryingGoDown = false;
        }
        if (!this.hasResetToZero && this.isTryingGoDown) {
            this.elevatorLeftLeader.power(-0.15);
            this.targetPosition = null;
        } else if (this.targetPosition != null) {
            double feedforward = this.getFeedforward(this.targetPosition);
            this.elevatorLeftLeader.MotionMagic(this.targetPosition.getSensorPosition(), feedforward);
        }

        if (this.targetPosition != null && this.getPosition().sensorPosition.lt(Level.L2.getSensorPosition()) && this.targetPosition.sensorPosition.gt(Level.L2.getSensorPosition())) {
            var notification = new Elastic.Notification().withTitle("MOVE DOWN INTAKE").withLevel(Elastic.Notification.NotificationLevel.WARNING).withDisplaySeconds(3);
            Elastic.sendNotification(notification);
        }
    }

    /**
     * Move the elevator to the specific position.
     *
     * @param position the target position
     */
    public void moveTo(Position position) {
        this.targetPosition = position;
    }

    /**
     * Move the elevator to the specific position.
     *
     * @param level the target level
     */
    public void moveTo(Level level) {
        this.moveTo(level.position);
    }

    /**
     * Get the feedforward of the elevator.
     *
     * @param position the target position
     * @return the feedforward
     */
    private double getFeedforward(Position position) {
        return this.getFeedforward(position.sensorPosition);
    }

    /**
     * Get the feedforward of the elevator.
     *
     * @param sensorPosition the target position
     * @return the feedforward
     */
    private double getFeedforward(Angle sensorPosition) {
        // Our elevator doesn't have a constant feedforward, so we just return 0.
        return 0;
    }

    /**
     * Brake the elevator.
     */
    public void brake() {
        this.elevatorLeftLeader.brake();
        this.targetPosition = this.getPosition();
    }

    /**
     * Get the current position of the elevator.
     *
     * @return the current position of the elevator
     */
    public Position getPosition() {
        return new Position(this.elevatorLeftLeader.getPosition());
    }

    /**
     * Get the command that moves the elevator up.
     *
     * @return the command that moves the elevator up
     */
    public Command getMoveUpCommand() {
        return runOnce(() -> {
            if (!this.hasResetToZero) {
                return;
            }
            if (this.targetPosition == null) {
                this.targetPosition = new Position(Units.Radians.of(0));
            }
            var targetSensorPosition = this.targetPosition.sensorPosition.plus(Units.Radians.of(5));
            this.moveTo(getMovablePosition(new Position(targetSensorPosition)));
        });
    }

    /**
     * Get the command that moves the elevator down.
     *
     * @return the command that moves the elevator down
     */
    public Command getMoveDownCommand() {
        return runOnce(() -> {
            if (!this.hasResetToZero) {
                return;
            }
            if (this.targetPosition == null) {
                this.targetPosition = new Position(Units.Radians.of(0));
            }
            var targetSensorPosition = this.targetPosition.sensorPosition.minus(Units.Radians.of(5));
            this.moveTo(getMovablePosition(new Position(targetSensorPosition)));
        });
    }

    /**
     * Get the command that moves the elevator to the specific position.
     *
     * @param position the target position
     * @return the command that moves the elevator to the specific position
     */
    public Command getMoveCommand(Position position) {
        return runOnce(() -> this.moveTo(position));
    }

    /**
     * Get the command that moves the elevator to the specific position.
     *
     * @param level the target level
     * @return the command that moves the elevator to the specific position
     */
    public Command getMoveCommand(Level level) {
        return runOnce(() -> this.moveTo(level));
    }

    /**
     * Get whether the elevator is at the bottom.
     *
     * @return whether the elevator is at the bottom
     */
    public boolean getAtBottomState() {
        return !this.limitSwitch.get();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Elevator");
        builder.setActuator(true);
        builder.setSafeState(this::brake);
        builder.addDoubleProperty("Position", () -> this.hasResetToZero ? this.getPosition().getSensorPosition().in(Units.Radians) : -1, (position) -> this.moveTo(new Position(Units.Radians.of(position))));
        builder.addDoubleProperty("Target Position", () -> this.targetPosition == null ? -1 : this.targetPosition.getSensorPosition().in(Units.Radians), (position) -> this.moveTo(new Position(Units.Radians.of(position))));
        builder.addBooleanProperty("At Bottom", this::getAtBottomState, null);
        SmartDashboard.putData("Move to Bottom", this.getMoveCommand(Level.BOTTOM));
        SmartDashboard.putData("Move to L1", this.getMoveCommand(Level.L1));
        SmartDashboard.putData("Ball L2", this.getMoveCommand(Level.L2B));
        SmartDashboard.putData("Move to L2", this.getMoveCommand(Level.L2));
        SmartDashboard.putData("Move to L3", this.getMoveCommand(Level.L3));
        SmartDashboard.putData("Ball L3", this.getMoveCommand(Level.L3B));
        SmartDashboard.putData("Move to L4", this.getMoveCommand(Level.L4));
        SmartDashboard.putData("Reset", Commands.runOnce(this::reset).ignoringDisable(true));
    }

    /**
     * Reset the elevator.
     */
    public void reset() {
        this.elevatorLeftLeader.resetPosition();
        this.targetPosition = null;
    }

    /**
     * Try to go down to the bottom.
     */
    public void tryGoDown() {
        this.hasResetToZero = false;
        this.isTryingGoDown = true;
    }

    public Position getTargetPosition() {
        return Objects.requireNonNullElseGet(this.targetPosition, () -> new Position(Units.Radians.of(0)));
    }

    /**
     * The predefined levels of the elevator.
     */
    public enum Level {
        BOTTOM(0), L1(0.5), L2(7.7), L2B(27), L3(27.5), L3B(48), L4(62);

        // The position of the level.
        private final Position position;

        /**
         * Construct a new Level.
         *
         * @param sensorPosition the sensor position of the level
         */
        Level(double sensorPosition) {
            this.position = new Position(Units.Radians.of(sensorPosition));
        }

        public Angle getSensorPosition() {
            return this.position.sensorPosition;
        }
    }

    /**
     * The position of the elevator.
     */
    public static class Position {
        // The sensor position of motor.
        private final Angle sensorPosition;

        /**
         * Construct a new Position.
         *
         * @param sensorPosition the sensor position of motor
         */
        public Position(Angle sensorPosition) {
            this.sensorPosition = sensorPosition;
        }

        /**
         * Get the sensor position.
         *
         * @return the sensor position
         */
        public Angle getSensorPosition() {
            return sensorPosition;
        }
    }
}

