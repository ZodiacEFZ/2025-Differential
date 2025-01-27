package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.libzodiac.drivetrain.ZDifferential;
import frc.libzodiac.util.Rotation2dSupplier;

import java.util.function.DoubleSupplier;

public class RobotContainer {
    private final CommandXboxController driver = new CommandXboxController(0);
    private final ZDifferential drivetrain;
    private final PowerDistribution powerDistribution = new PowerDistribution();

    public RobotContainer() {
        ZDifferential.Config differentialConfig = new ZDifferential.Config();
        differentialConfig.ROBOT_WIDTH = 0.762;
        differentialConfig.MAX_SPEED = 3;
        differentialConfig.MAX_ANGULAR_SPEED = 2 * Math.PI;
        differentialConfig.GEAR_RATIO = 8.46;
        differentialConfig.WHEEL_RADIUS = 0.0508;

        differentialConfig.leftLeader = 1;
        differentialConfig.leftFollower = 2;
        differentialConfig.rightLeader = 3;
        differentialConfig.rightFollower = 4;
        differentialConfig.leftEncoder = 5;
        differentialConfig.rightEncoder = 6;

        differentialConfig.gyro = 0;

        differentialConfig.pidController = new PIDController(1, 0, 0);
        differentialConfig.headingController = new PIDController(0.4, 0.01, 0.01);
        differentialConfig.headingController.setIZone(Math.PI / 4);

        this.drivetrain = new ZDifferential(differentialConfig);

        // Configure the button bindings
        this.configureButtonBindings();
        this.setDirectAngle(true);
        this.setDriveCommand();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        driver.a().onTrue(Commands.none());
        driver.b().onTrue(Commands.none());
        driver.x().onTrue(Commands.none());
        driver.y().onTrue(Commands.runOnce(drivetrain::zeroHeading));
        driver.leftBumper().onTrue(Commands.none());
        driver.rightBumper().onChange(Commands.runOnce(this::toggleDirectAngle));
        driver.start().onTrue(Commands.none());
        driver.back().onTrue(Commands.none());
    }

    public void setDirectAngle(boolean directAngle) {
        this.drivetrain.setDirectAngle(directAngle);
        this.setDriveCommand();
    }

    private void setDriveCommand() {
        DoubleSupplier velocitySupplier = () -> {
            double forward = MathUtil.applyDeadband(driver.getRightTriggerAxis(), 0.05);
            double reverse = MathUtil.applyDeadband(driver.getLeftTriggerAxis(), 0.05);
            if (forward != 0 && reverse != 0) {
                double velocity = forward - reverse;
                if (Math.abs(velocity) > 0.5) {
                    return velocity;
                }
                return 0;
            }
            return forward - reverse;
        };

        /*
          Converts driver input into a ChassisSpeeds that is controlled by angular velocity.
         */
        var angularVelocityInput = new ZDifferential.DifferentialInputStream(drivetrain, velocitySupplier).withRotation(
                driver::getLeftX).deadband(0.05);

        /*
          Clone's the angular velocity input stream and converts it to a direct angle input stream.
         */
        var directAngleInput = new ZDifferential.DifferentialInputStream(drivetrain, velocitySupplier).withHeading(
                new Rotation2dSupplier(() -> -driver.getRightX(), () -> -driver.getRightY())).deadband(0.05);

        /*
          Direct angle input can only be used in field centric mode.
         */
        this.drivetrain.setDefaultCommand(
                this.drivetrain.driveCommand(directAngleInput, angularVelocityInput, this.drivetrain.getDirectAngle()));
    }

    public void toggleDirectAngle() {
        this.drivetrain.toggleDirectAngle();
        this.setDriveCommand();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
        //todo
    }

    public void setMotorBrake(boolean brake) {
        drivetrain.setMotorBrake(brake);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("Voltage", powerDistribution.getVoltage());
        SmartDashboard.putData("Drivetrain", drivetrain);
    }
}
