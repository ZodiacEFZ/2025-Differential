package frc.robot.subsystem;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.libzodiac.hardware.TalonSRXMotor;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public class Outtake {
    // The motor that controls the outtake.
    TalonSRXMotor outtakeMotor = new TalonSRXMotor(11);
    // The servos that control the outtake.
    Servo leftServo = new Servo(0);
    Servo rightServo = new Servo(1);
    // The state of the outtake.
    boolean isUp;
    boolean isOpen;

    /**
     * Construct a new Outtake.
     */
    public Outtake() {
        this.outtakeMotor.factoryDefault();
        this.outtakeMotor.setInverted(false);
        this.outtakeMotor.setPID(0.1, 0, 0);

        this.close();
        this.up();
    }

    /**
     * Open the outtake.
     */
    public void open() {
        this.leftServo.set(0.1);
        this.rightServo.set(0.95);
        this.isOpen = true;
    }

    /**
     * Close the outtake.
     */
    public void close() {
        this.leftServo.set(0.7);
        this.rightServo.set(0.35);
        this.isOpen = false;
    }

    /**
     * Switch the state of the outtake.
     */
    public void switchOuttakeState() {
        if (this.isOpen) {
            this.close();
        } else {
            this.open();
        }
    }

    /**
     * Get the command to open the outtake.
     *
     * @return The command to open the outtake.
     */
    public Command getOpenCommand() {
        return runOnce(this::open);
    }

    /**
     * Get the command to close the outtake.
     *
     * @return The command to close the outtake.
     */
    public Command getCloseCommand() {
        return runOnce(this::close);
    }

    /**
     * Get the command to switch the state of the outtake.
     *
     * @return The command to switch the state of the outtake.
     */
    public Command getSwitchOuttakeStateCommand() {
        return runOnce(this::switchOuttakeState);
    }

    /**
     * Move the outtake up.
     */
    public void up() {
        runOnce(() -> this.outtakeMotor.power(0.3)).repeatedly().withTimeout(1).finallyDo(this.outtakeMotor::brake).schedule();
        isUp = true;
    }

    /**
     * Move the outtake down.
     */
    public void down() {
        runOnce(() -> this.outtakeMotor.power(-0.3)).repeatedly().withTimeout(0.3).finallyDo(this.outtakeMotor::brake).schedule();
        isUp = false;
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
