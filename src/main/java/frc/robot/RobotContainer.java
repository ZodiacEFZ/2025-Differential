package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.libzodiac.drivetrain.Differential;
import frc.libzodiac.drivetrain.PathPlanner;
import frc.libzodiac.hardware.Limelight;
import frc.libzodiac.util.CommandUtil;
import frc.libzodiac.util.Rotation2dSupplier;
import frc.robot.subsystem.Elevator;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Outtake;

import java.util.function.DoubleSupplier;

public class RobotContainer {
    // The driver's controller
    private final CommandXboxController driver = new CommandXboxController(0);
    // The driver's second controller
    private final CommandXboxController controller = new CommandXboxController(1);
    // The robot's subsystems
    private final Differential drivetrain;
    private final Limelight limelight;
    private final PowerDistribution powerDistribution = new PowerDistribution();
    private final Elevator elevator = new Elevator();
    private final Intake intake = new Intake();
    private final Outtake outtake = new Outtake();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        Differential.Config differentialConfig = new Differential.Config();
        differentialConfig.ROBOT_WIDTH = 0.762;
        differentialConfig.MAX_SPEED = 3;
        differentialConfig.MAX_ANGULAR_VELOCITY = Math.PI;
        differentialConfig.WHEEL_RADIUS = 0.0762;

        differentialConfig.leftLeader = 1;
        differentialConfig.leftFollower = 2;
        differentialConfig.rightLeader = 3;
        differentialConfig.rightFollower = 4;

        differentialConfig.leftLeaderInverted = false;
        differentialConfig.leftFollowerInverted = false;
        differentialConfig.rightLeaderInverted = true;
        differentialConfig.rightFollowerInverted = true;

        differentialConfig.leftEncoderPhase = true;
        differentialConfig.rightEncoderPhase = true;

        differentialConfig.gyro = 0;

        differentialConfig.pidController = new PIDController(0.1, 0.002, 0.5);
        differentialConfig.headingController = new PIDController(0.5, 0.025, 0.2);
        differentialConfig.headingController.setIZone(Math.PI / 8);

        this.drivetrain = new Differential(differentialConfig, new Pose2d()); // TODO: Set initial pose

        PathPlanner.initInstance(this.drivetrain);

        // Configure the button bindings
        this.configureButtonBindings();
        this.drivetrain.setDirectAngle(true);
        this.setDriveCommand();

        // Build an auto chooser
        autoChooser = PathPlanner.getInstance().buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Start the camera server
        // Disable the warning
        //noinspection resource
        CameraServer.startAutomaticCapture();

        this.limelight = new Limelight(drivetrain);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        this.driver.leftBumper().onChange(Commands.runOnce(this.drivetrain::toggleSlowMode));
        this.driver.rightBumper().onChange(Commands.runOnce(this::toggleDirectAngle));
        this.driver.a().onTrue(Commands.runOnce(this::zeroHeading));

        this.controller.a().onTrue(this.outtake.getSwitchOuttakeStateCommand());
        this.controller.b().onTrue(this.intake.getIntakeCommand()).onFalse(this.intake.getStopCommand());
        this.controller.x().onTrue(this.intake.getOuttakeCommand()).onFalse(this.intake.getStopCommand());
        this.controller.y().onTrue(this.outtake.getSwitchUpStateCommand());
        this.controller.povUp().onTrue(this.elevator.getMoveUpCommand());
        this.controller.povDown().onTrue(this.elevator.getMoveDownCommand());
        this.controller.povLeft().onTrue(Commands.runOnce(this.elevator::tryGoDown));
    }

    /**
     * Set the default command for the drivetrain.
     */
    private void setDriveCommand() {
        DoubleSupplier velocitySupplier = () -> {
            double forward = MathUtil.applyDeadband(this.driver.getRightTriggerAxis(), 0.05);
            double reverse = MathUtil.applyDeadband(this.driver.getLeftTriggerAxis(), 0.05);
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
        var angularVelocityInput = new Differential.InputStream(this.drivetrain, velocitySupplier).rotation(
                () -> -this.driver.getLeftX()).deadband(0.05);

        /*
          Clone's the angular velocity input stream and converts it to a direct angle input stream.
         */
        var directAngleInput = new Differential.InputStream(this.drivetrain, velocitySupplier).heading(
                new Rotation2dSupplier(() -> -this.driver.getLeftY(), () -> -this.driver.getLeftX())).deadband(0.05);

        /*
          Direct angle input can only be used in field centric mode.
         */
        this.drivetrain.setDefaultCommand(this.drivetrain.getDriveCommand(directAngleInput, angularVelocityInput,
                this.drivetrain::getDirectAngle));
    }

    private void zeroHeading() {
        this.drivetrain.zeroHeading();
        CommandUtil.rumbleController(this.driver.getHID(), 0.5, 0.5);
    }

    public void toggleDirectAngle() {
        this.drivetrain.toggleDirectAngle();
        CommandUtil.rumbleController(this.driver.getHID(), 0.5, 0.5);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return this.autoChooser.getSelected();
    }

    public void setMotorBrake(boolean brake) {
        this.drivetrain.setMotorBrake(brake);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("Voltage", this.powerDistribution.getVoltage());
        SmartDashboard.putData("Drivetrain", this.drivetrain);
        SmartDashboard.putData("Field", this.drivetrain.getField());
        SmartDashboard.putData("Elevator", this.elevator);
    }

    public CommandXboxController getDriverController() {
        return this.driver;
    }
}
