package frc.robot.subsystem;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
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
    boolean hasResetToZero = false;

    public Elevator() {
        this.elevatorLeftLeader.factoryDefault();
        this.elevatorLeftFollower.factoryDefault();
        this.elevatorRightFollower1.factoryDefault();
        this.elevatorRightFollower2.factoryDefault();
        this.elevatorLeftLeader.setMotionMagicConfig(0.1, 0.005, 1, 0.125, Units.RadiansPerSecond.of(12 * Math.PI),
                Units.RadiansPerSecondPerSecond.of(12 * Math.PI), 3);
        this.elevatorLeftLeader.setMaxIntergralAccum(27500);
        this.elevatorLeftLeader.setPeakOutput(0.5);

        this.elevatorLeftLeader.setInverted(false);
        this.elevatorLeftLeader.setPhase(false);

        this.elevatorLeftFollower.follow(elevatorLeftLeader);
        this.elevatorRightFollower1.follow(elevatorLeftFollower, true);
        this.elevatorRightFollower2.follow(elevatorLeftLeader, true);

        this.elevatorLeftLeader.shutdown();
        Timer.delay(3);
        this.elevatorLeftLeader.brake();
        this.elevatorLeftLeader.setBrakeMode(true);
        this.elevatorLeftLeader.resetPosition();
        this.targetPosition = null;
    }

    private static Position getMovablePosition(Position target) {//Limits
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
        if (this.atBottom()) {
            this.elevatorLeftLeader.brake();
            this.elevatorLeftLeader.resetPosition();
            this.hasResetToZero = true;
        }
        if (!this.hasResetToZero) {
            this.elevatorLeftLeader.power(-0.1);
        } else if (this.targetPosition != null) {
            double feedforward = this.getFeedforward(this.targetPosition);
            this.elevatorLeftLeader.MotionMagic(this.targetPosition.getSensorPosition(), feedforward);
        }
    }

    public void moveTo(Position position) {
        this.targetPosition = position;
    }

    public void moveTo(Level level) {
        this.moveTo(level.position);
    }

    private double getFeedforward(Position position) {
        return this.getFeedforward(position.sensorPosition);
    }

    private double getFeedforward(Angle sensorPosition) {
        // Our elevator doesn't have a constant feedforward, so we just return 0.
        return 0;
    }

    public void brake() {
        this.elevatorLeftLeader.brake();
        this.targetPosition = this.getPosition();
    }

    public Position getPosition() {
        return new Position(this.elevatorLeftLeader.getPosition());
    }

    public Command getMoveUpCommand() {//Elevator up by 5(pos)
        var target = new Position(this.getPosition().getSensorPosition().plus(Units.Radians.of(5)));
        return runOnce(() -> this.moveTo(getMovablePosition(target)));
    }

    public Command getMoveDownCommand() {//Elevator down by 5(pos)
        var target = new Position(this.getPosition().getSensorPosition().minus(Units.Radians.of(5)));
        return runOnce(() -> this.moveTo(getMovablePosition(target)));
    }

    public Command getMoveCommand(Position position) {
        return runOnce(() -> this.moveTo(position));
    }

    public Command getMoveCommand(Level level) {
        return runOnce(() -> this.moveTo(level));
    }

    public boolean atBottom() {
        return !this.limitSwitch.get();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Elevator");
        builder.setActuator(true);
        builder.setSafeState(this::brake);
        builder.addDoubleProperty("Position", () -> this.hasResetToZero ? this.getPosition().getSensorPosition().in(Units.Radians) : -1, (position) -> this.moveTo(new Position(Units.Radians.of(position))));
        builder.addBooleanProperty("At Bottom", this::atBottom, null);
        SmartDashboard.putData("Move to Bottom", this.getMoveCommand(Level.BOTTOM));
        SmartDashboard.putData("Move to L1", this.getMoveCommand(Level.L1));
        SmartDashboard.putData("Ball L2", this.getMoveCommand(Level.L2B));
        SmartDashboard.putData("Move to L2", this.getMoveCommand(Level.L2));
        SmartDashboard.putData("Move to L3", this.getMoveCommand(Level.L3));
        SmartDashboard.putData("Ball L3", this.getMoveCommand(Level.L3B));
        SmartDashboard.putData("Move to L4", this.getMoveCommand(Level.L4));
        SmartDashboard.putData("Reset", Commands.runOnce(this::reset).ignoringDisable(true));
    }

    public void reset() {
        this.elevatorLeftLeader.resetPosition();
        this.targetPosition = null;
    }

    public void tryGoDown() {
        this.hasResetToZero = false;
    }

    public enum Level {
        BOTTOM(0), L1(10), L2(10), L2B(30.6), L3(35.2), L3B(50.6), L4(63.2);

        private final Position position;

        Level(double sensorPosition) {
            this.position = new Position(Units.Radians.of(sensorPosition));
        }
    }

    public static class Position {
        private final Angle sensorPosition;

        public Position(Angle sensorPosition) {
            this.sensorPosition = sensorPosition;
        }

        private Angle getSensorPosition() {
            return sensorPosition;
        }
    }
}

