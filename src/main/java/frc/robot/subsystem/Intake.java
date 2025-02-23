package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.hardware.TalonFXMotor;

public class Intake extends SubsystemBase {
    // The motor for the intake.
    TalonFXMotor leftMotor = new TalonFXMotor(10);
    TalonFXMotor rightMotor = new TalonFXMotor(9);

    /**
     * Construct a new Intake.
     */
    public Intake() {
        this.leftMotor.factoryDefault();
        this.rightMotor.factoryDefault();

        this.leftMotor.setInverted(false);
        this.rightMotor.setInverted(true);

        this.leftMotor.setPID(0.1, 0, 0);
        this.rightMotor.setPID(0.1, 0, 0);
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
}
