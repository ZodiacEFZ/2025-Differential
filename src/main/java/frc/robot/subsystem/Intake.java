package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.hardware.TalonFXMotor;

public class Intake extends SubsystemBase {
    TalonFXMotor leftMotor = new TalonFXMotor(10);
    TalonFXMotor rightMotor = new TalonFXMotor(9);

    public Intake() {
        this.leftMotor.factoryDefault();
        this.rightMotor.factoryDefault();

        this.leftMotor.setInverted(false);
        this.rightMotor.setInverted(true);

        this.leftMotor.setPID(0.1, 0, 0);
        this.rightMotor.setPID(0.1, 0, 0);
    }

    public void intake() {
        this.leftMotor.power(0.3);
        this.rightMotor.power(0.3);
    }

    public void outtake() {
        this.leftMotor.power(-0.3);
        this.rightMotor.power(-0.3);
    }

    public void stop() {
        this.leftMotor.brake();
        this.rightMotor.brake();
    }

    public Command getIntakeCommand() {
        return runOnce(this::intake);
    }

    public Command getStopCommand() {
        return runOnce(this::stop);
    }

    public Command getOuttakeCommand() {
        return runOnce(this::outtake);
    }

    public Command getIntakeCommand(double seconds) {
        return runOnce(this::intake).repeatedly().withTimeout(seconds).finallyDo(this::stop);
    }

    public void intake(double seconds) {
        this.getIntakeCommand(seconds).schedule();
    }
}
