package frc.robot.subsystem;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.libzodiac.hardware.TalonSRXMotor;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public class Outtake {
    // The servos that control the outtake.
    Servo leftServo = new Servo(0);
    Servo rightServo = new Servo(1);
    // The state of the outtake.
    boolean isOpen;

    /**
     * Construct a new Outtake.
     */
    public Outtake() {
        this.close();
    }

    /**
     * Open the outtake.
     */
    public void open() {
        this.leftServo.set(0.05);
        this.rightServo.set(0.55);
        this.isOpen = true;
    }

    /**
     * Close the outtake.
     */
    public void close() {
        this.leftServo.set(0.6);
        this.rightServo.set(0);
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
}
