package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
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
import frc.libzodiac.hardware.Pigeon;
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
    private final CommandXboxController operator = new CommandXboxController(1);
    // The robot's subsystems
    private final Differential drivetrain;
    private final PowerDistribution powerDistribution = new PowerDistribution();
    private final Elevator elevator = new Elevator();
    private final Intake intake = new Intake();
    private final Outtake outtake = new Outtake();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Configure the drivetrain
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

        differentialConfig.gyro = new Pigeon(0);

        differentialConfig.pidController = new PIDController(1, 0.0005, 0.02);
        differentialConfig.headingController = new PIDController(0.5, 0.025, 0.2);
        differentialConfig.headingController.setIZone(Math.PI / 8);

        this.drivetrain = new Differential(differentialConfig, new Pose2d()); // TODO: Set initial pose

        // Initialize PathPlanner
        PathPlanner.initInstance(this.drivetrain);

        // Configure the button bindings
        this.configureButtonBindings();
        this.drivetrain.setDirectAngle(false);
        this.drivetrain.setSlowMode(true);
        this.setDriveCommand();

        // Build an auto chooser
        autoChooser = PathPlanner.getInstance().buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Start the camera server
        // Disable the warning
        //noinspection resource
        CameraServer.startAutomaticCapture();
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

        this.operator.a().onTrue(this.outtake.getSwitchOuttakeStateCommand());
        this.operator.b().onTrue(this.intake.getIntakeCommand()).onFalse(this.intake.getStopCommand());
        this.operator.x().onTrue(this.intake.getOuttakeCommand()).onFalse(this.intake.getStopCommand());
        this.operator.y().onTrue(this.intake.getSwitchUpStateCommand());
        this.operator.povUp().onTrue(this.elevator.getMoveUpCommand());
        this.operator.povDown().onTrue(this.elevator.getMoveDownCommand());
        this.operator.povLeft().onTrue(Commands.runOnce(this.elevator::tryGoDown));
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

    /**
     * Zero the heading of the robot.
     */
    private void zeroHeading() {
        this.drivetrain.zeroHeading();
        CommandUtil.rumbleController(this.driver.getHID(), 0.5, 0.5);
    }

    /**
     * Toggle direct angle mode.
     */
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

    /**
     * Set the motor brake mode.
     *
     * @param brake whether to brake the motors
     */
    public void setMotorBrake(boolean brake) {
        this.drivetrain.setMotorBrake(brake);
    }

    /**
     * Update the dashboard.
     */
    public void updateDashboard() {
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("Voltage", this.powerDistribution.getVoltage());
        SmartDashboard.putData("Drivetrain", this.drivetrain);
        SmartDashboard.putData("Field", this.drivetrain.getField());
        SmartDashboard.putData("Elevator", this.elevator);
    }

    /**
     * Get the driver's controller.
     *
     * @return the driver's controller
     */
    public CommandXboxController getDriverJoystick() {
        return this.driver;
    }

    /**
     * Get the operator's controller.
     *
     * @return the operator's controller
     */
    public CommandXboxController getOperatorJoystick() {
        return this.operator;
    }

    public Command getMoveIntakeDownCommand() {
        return this.intake.getDownCommand();
    }

    public Command getMoveIntakeUpCommand() {
        return this.intake.getUpCommand();
    }

    public Command getLeaveCommand() {
        return Commands.runOnce(() -> this.drivetrain.drive(-0.5, 0), this.drivetrain).repeatedly()
                .withTimeout(10).finallyDo(() -> this.drivetrain.drive(0, 0));
    }

    public Angle getElevatorSensorPosition() {
        return this.elevator.getPosition().getSensorPosition();
    }

    public Angle getTargetPosition() {
        return this.elevator.getTargetPosition().getSensorPosition();
    }

    public Command getSetDrivetrainSlowCommand(boolean slowMode) {
        return Commands.runOnce(() -> this.drivetrain.setSlowMode(slowMode));
    }
}
