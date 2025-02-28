package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.hardware.TalonFXMotor;
import frc.libzodiac.hardware.TalonSRXMotor;

public class Intake extends SubsystemBase {
    // The motor for the intake.
    TalonFXMotor leftMotor = new TalonFXMotor(10);
    TalonFXMotor rightMotor = new TalonFXMotor(9);

    // The motor that controls the intake.
    TalonSRXMotor intakeMotor = new TalonSRXMotor(11);

    // The state of the intake.
    boolean isUp;

    /**
     * Construct a new Intake.
     */
    public Intake() {
        this.leftMotor.factoryDefault();
        this.rightMotor.factoryDefault();
        this.intakeMotor.factoryDefault();

        this.leftMotor.setInverted(false);
        this.rightMotor.setInverted(true);
        this.intakeMotor.setInverted(false);

        this.leftMotor.setPID(0.1, 0, 0);
        this.rightMotor.setPID(0.1, 0, 0);
        this.intakeMotor.setPID(0.1, 0, 0);

        this.isUp = true;
    }

    /**
     * Intake the ball.
     */
    public void intake() {
        this.leftMotor.power(0.3);
        this.rightMotor.power(0.3);
    }

    /**
     * Outtake the ball.
     */
    public void outtake() {
        this.leftMotor.power(-0.3);
        this.rightMotor.power(-0.3);
    }

    /**
     * Stop the intake.
     */
    public void stop() {
        this.leftMotor.brake();
        this.rightMotor.brake();
    }

    /**
     * Get the intake command.
     *
     * @return the intake command
     */
    public Command getIntakeCommand() {
        return runOnce(this::intake);
    }

    /**
     * Get the stop command.
     *
     * @return the stop command
     */
    public Command getStopCommand() {
        return runOnce(this::stop);
    }

    /**
     * Get the outtake command
     *
     * @return the outtake command
     */
    public Command getOuttakeCommand() {
        return runOnce(this::outtake);
    }

    /**
     * Get the intake command with a timeout.
     *
     * @param seconds The time to intake in seconds.
     * @return the intake command with a timeout
     */
    public Command getIntakeCommand(double seconds) {
        return runOnce(this::intake).repeatedly().withTimeout(seconds).finallyDo(this::stop);
    }

    /**
     * Intake the ball with a timeout.
     *
     * @param seconds The time to intake in seconds.
     */
    public void intake(double seconds) {
        this.getIntakeCommand(seconds).schedule();
    }

    /**
     * Move the outtake up.
     */
    public void up() {
        runOnce(() -> this.intakeMotor.power(-0.3)).repeatedly().withTimeout(1).finallyDo(this.intakeMotor::brake).schedule();
        this.isUp = true;
    }

    /**
     * Move the outtake down.
     */
    public void down() {
        runOnce(() -> this.intakeMotor.power(0.3)).repeatedly().withTimeout(0.5).finallyDo(this.intakeMotor::brake).schedule();
        this.isUp = false;
    }

    /**
     * Switch the state of the outtake.
     */
    public void switchUpState() {
        if (this.isUp) {
            this.down();
        } else {
            this.up();
        }
    }

    /**
     * Get the command to move the outtake up.
     *
     * @return The command to move the outtake up.
     */
    public Command getUpCommand() {
        return Commands.runOnce(this::up);
    }

    /**
     * Get the command to move the outtake down.
     *
     * @return The command to move the outtake down.
     */
    public Command getDownCommand() {
        return Commands.runOnce(this::down);
    }

    /**
     * Get the command to switch the state of the outtake.
     *
     * @return The command to switch the state of the outtake.
     */
    public Command getSwitchUpStateCommand() {
        return Commands.runOnce(this::switchUpState);
    }
}
