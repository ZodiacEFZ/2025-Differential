package frc.robot.subsystem;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.libzodiac.hardware.TalonSRXMotor;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public class Outtake {
    TalonSRXMotor outtakeMotor = new TalonSRXMotor(11);
    Servo leftServo = new Servo(0);
    Servo rightServo = new Servo(1);
    boolean isUp;
    boolean isOpen;

    public Outtake() {
        this.outtakeMotor.factoryDefault();
        this.outtakeMotor.setInverted(false);
        this.outtakeMotor.setPID(0.1, 0, 0);

        this.close();
        this.down();
    }

    public void open() {
        this.leftServo.set(0.1);
        this.rightServo.set(0.95);
        this.isOpen = true;
    }

    public void close() {
        this.leftServo.set(0.7);
        this.rightServo.set(0.35);
        this.isOpen = false;
    }

    public void switchOuttakeState() {
        if (this.isOpen) {
            this.close();
        } else {
            this.open();
        }
    }

    public Command getOpenCommand() {
        return runOnce(this::open);
    }

    public Command getCloseCommand() {
        return runOnce(this::close);
    }

    public Command getSwitchOuttakeStateCommand() {
        return runOnce(this::switchOuttakeState);
    }

    //todo: use voltage or current
    public void up() {
        runOnce(() -> this.outtakeMotor.power(0.3)).repeatedly().withTimeout(1).finallyDo(this.outtakeMotor::brake).schedule();
        isUp = true;
    }

    public void down() {
        runOnce(() -> this.outtakeMotor.power(-0.3)).repeatedly().withTimeout(0.3).finallyDo(this.outtakeMotor::brake).schedule();
        isUp = false;
    }

    public void switchUpState() {
        if (this.isUp) {
            this.down();
        } else {
            this.up();
        }
    }

    public Command getUpCommand() {
        return Commands.runOnce(this::up);
    }

    public Command getDownCommand() {
        return Commands.runOnce(this::down);
    }

    public Command getSwitchUpStateCommand() {
        return Commands.runOnce(this::switchUpState);
    }
}
