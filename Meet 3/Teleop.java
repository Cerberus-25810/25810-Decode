package org.firstinspires.ftc.teamcode.Meet3;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import static org.firstinspires.ftc.teamcode.Meet3.BlueSideMeet3.AutoEndPose;

@TeleOp(name="Meet3Teleop", group="TeleOp")
public class Meet3Teleop extends OpMode {

    // Drive motors
    private DcMotor rightBack;
    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor leftBack;

    private Pose curPose;

    // Goal position coordinates (13, 135)
    private static final double GOAL_X = 13.0;
    private static final double GOAL_Y = 135.0;

    private final Pose startPose = new Pose(51,91,Math.toRadians(143));

    // Scoring motors
    private DcMotorEx Shooter;
    private DcMotorEx Shooter2;

    private DcMotor Intake;
    private DcMotor Control;

    // Servos
    private CRServo LeftServo;
    private CRServo RightServo;
    private ServoImplEx Hood;  // Hood servo for auto-aiming

    // Auto-aiming toggle
    private boolean autoAimEnabled = true;
    private boolean xPressed = false;

    // Store last calculated values to prevent jumps
    private double lastShooterVelocity = 1130;
    private double lastHoodPosition = 0.5;

    // Timer for servo delay
    private ElapsedTime triggerTimer = new ElapsedTime();
    private ElapsedTime rightTriggerTimer = new ElapsedTime();
    private boolean triggerPressed = false;
    private boolean rightTriggerPressed = false;
    private int controlTargetPosition = 0;
    private boolean controlStopped = false;


    @Override
    public void init() {
        // Initialize drive motors
        follower = Constants.createFollower(hardwareMap);
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");

        // Initialize scoring motors
        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Shooter2 = hardwareMap.get(DcMotorEx.class,"Shooter2");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Control = hardwareMap.get(DcMotor.class, "Control");

        // Set Control motor to use encoder for position control
        Control.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Control.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize servos
        LeftServo = hardwareMap.get(CRServo.class, "LeftServo");
        RightServo = hardwareMap.get(CRServo.class, "RightServo");
        Hood = hardwareMap.get(ServoImplEx.class, "Hood");

        // Configure Axon servo for extended PWM range
        Hood.setPwmRange(new PwmControl.PwmRange(500, 2500));

        Hood.setPosition(0);
        // Set servo directions - adjust if servos move in opposite directions
        LeftServo.setDirection(CRServo.Direction.FORWARD);
        RightServo.setDirection(CRServo.Direction.REVERSE);

        // Set motor directions (adjust these based on your robot's configuration)
        Shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        Shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        Control.setDirection(DcMotorSimple.Direction.REVERSE);


        // pidf stuff
        PIDFCoefficients SHOOTERpidfCoefficients = new PIDFCoefficients(0.005,0,0,15.5);
        Shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,SHOOTERpidfCoefficients);
        Shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,SHOOTERpidfCoefficients);

        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        // Set starting pose from auto
        follower.setPose(AutoEndPose);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Goal Position", "X: %.1f, Y: %.1f", GOAL_X, GOAL_Y);
    }

    @Override
    public void loop() {
        // Update Pedro Pathing localization
        follower.update();
        curPose = follower.getPose();

        // Get current robot coordinates
        double robotX = curPose.getX();
        double robotY = curPose.getY();

        // Calculate distance to goal using distance formula: sqrt((x2-x1)^2 + (y2-y1)^2)
        double deltaX = GOAL_X - robotX;
        double deltaY = GOAL_Y - robotY;
        double distanceToGoal = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        // GAMEPAD 1 - DRIVE CONTROLS
        double drive = -gamepad1.left_stick_y;  // Forward/backward
        double strafe = gamepad1.left_stick_x;   // Left/right strafing
        double turn = gamepad1.right_stick_x * 0.8;    // Turning (reduced to 80% speed)

        // Calculate mecanum drive powers
        double leftFrontPower = drive + strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double leftBackPower = drive - strafe + turn;
        double rightBackPower = drive + strafe - turn;

        // Normalize powers if any exceed 1.0
        double maxPower = Math.max(Math.abs(leftFrontPower),
                Math.max(Math.abs(rightFrontPower),
                        Math.max(Math.abs(leftBackPower),
                                Math.abs(rightBackPower))));

        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;
        }

        // Set drive motor powers
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

        // X button - Toggle auto-aim
        if (gamepad1.x && !xPressed) {
            xPressed = true;
            autoAimEnabled = !autoAimEnabled;

            // When toggling, preserve current values to prevent jumps
            if (!autoAimEnabled) {
                // Switching to manual - store the current auto values
                lastShooterVelocity = calculateShooterVelocity(distanceToGoal);
                lastHoodPosition = calculateHoodPosition(distanceToGoal);
            }
        } else if (!gamepad1.x) {
            xPressed = false;
        }

        // GAMEPAD 2 - SCORING CONTROLS
        // Calculate shooter velocity and hood position based on distance
        double shooterVelocity;
        double hoodPosition;

        if (autoAimEnabled) {
            // Auto-aim: calculate based on regression functions
            shooterVelocity = calculateShooterVelocity(distanceToGoal);
            hoodPosition = calculateHoodPosition(distanceToGoal);

            // Store these values for smooth transition
            lastShooterVelocity = shooterVelocity;
            lastHoodPosition = hoodPosition;
        } else {
            // Manual mode: use last known values to prevent jumping
            shooterVelocity = lastShooterVelocity;
            hoodPosition = lastHoodPosition;
        }

        // Set shooter velocity and hood position
        Shooter.setVelocity(shooterVelocity);
        Shooter2.setVelocity(shooterVelocity);
        Hood.setPosition(hoodPosition);
        Intake.setPower(1);

        // Default servo power (will be overridden by buttons)
        double servoSpeed = -1.0;

        // Default Control motor power - always running at 0.3
        Control.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Control.setPower(0.3);

        // Right trigger - stop motor first, then move back 500 ticks to lock ball
        if (gamepad2.right_trigger > 0.1) {
            if (!rightTriggerPressed) {
                // Trigger just pressed, stop the motor first
                rightTriggerTimer.reset();
                Control.setPower(0.0);
                rightTriggerPressed = true;
                controlStopped = false;
            }

            // After motor stops briefly, move it back 500 ticks
            if (rightTriggerTimer.milliseconds() > 100 && !controlStopped) {
                controlTargetPosition = Control.getCurrentPosition() - 10000;
                Control.setTargetPosition(controlTargetPosition);
                Control.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Control.setPower(0.3);
                controlStopped = true;
            }

            // Keep motor stopped once it reaches target
            if (controlStopped && Math.abs(Control.getCurrentPosition() - controlTargetPosition) < 10) {
                Control.setPower(0.0);
            }
        } else {
            if (rightTriggerPressed) {
                // Trigger released, switch back to normal running mode
                Control.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Control.setPower(0.3);
                rightTriggerPressed = false;
                controlStopped = false;
            }
        }

        // B button - manual control of Control motor at max speed
        if (gamepad2.b) {
            Control.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Control.setPower(1.0);
        }

        // Left trigger - Control motor and servo sequence
        if (gamepad2.left_trigger > 0.1) {
            if (!triggerPressed) {
                // Trigger just pressed, start timer
                triggerTimer.reset();
                triggerPressed = true;
            }

            // Set Control motor to full power
            Control.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Control.setPower(1);

            // After 0.1 seconds, start running servos continuously in positive direction
            if (triggerTimer.seconds() >= 0.1) {
                servoSpeed = 1.0;
            }
        } else {
            // Trigger released
            if (triggerPressed) {
                triggerPressed = false;
            }
        }

        // Servo control with A button (independent control)
        if (gamepad2.a) {
            servoSpeed = 1.0;
        }

        // Set servo speeds once at the end
        LeftServo.setPower(servoSpeed);
        RightServo.setPower(servoSpeed);

        // Telemetry for debugging
        telemetry.addData("=== POSITION DATA ===", "");
        telemetry.addData("Robot Position", "X: %.1f, Y: %.1f", robotX, robotY);
        telemetry.addData("Distance to Goal", "%.2f inches", distanceToGoal);
        telemetry.addData("", "");
        telemetry.addData("=== AUTO-AIM ===", "");
        telemetry.addData("Auto-Aim", autoAimEnabled ? "ENABLED (X to disable)" : "DISABLED (X to enable)");
        telemetry.addData("Shooter Velocity", "%.0f", shooterVelocity);
        telemetry.addData("Hood Position", "%.3f", hoodPosition);
        telemetry.addData("", "");
        telemetry.addData("=== DRIVE ===", "");
        telemetry.addData("Drive Power", "LF: %.2f, RF: %.2f, LB: %.2f, RB: %.2f",
                leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        telemetry.addData("", "");
        telemetry.addData("=== SCORING ===", "");
        telemetry.addData("Servos Active", gamepad2.a);
        telemetry.addData("Trigger Timer", "%.2f seconds", triggerTimer.seconds());
        telemetry.addData("Control Position", Control.getCurrentPosition());
        telemetry.addData("Control Target", controlTargetPosition);
        telemetry.update();
    }

    /**
     * Calculate shooter velocity based on distance to goal
     * Logarithmic regression: y = -493.52293 + 374.78968 * ln(x)
     */
    private double calculateShooterVelocity(double distance) {
        // Prevent ln(0) or negative values
        if (distance < 1) distance = 1;

        double velocity = -493.52293 + 374.78968 * Math.log(distance);

        // Clamp velocity to safe range
        if (velocity < 800) velocity = 800;
        if (velocity > 2000) velocity = 2000;

        return velocity;
    }

    /**
     * Calculate hood position based on distance to goal
     * Sigmoid regression: y = 0.24 / (1 + e^(-(0.596021x - 33.74169)))
     */
    private double calculateHoodPosition(double distance) {
        double exponent = -(0.596021 * distance - 33.74169);
        double position = 0.24 / (1 + Math.exp(exponent));

        // Clamp to servo range (0.0 to 1.0)
        if (position < 0.0) position = 0.0;
        if (position > 1.0) position = 1.0;

        return position;
    }
}
