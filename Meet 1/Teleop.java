package org.firstinspires.ftc.teamcode.Meet1.FinalCodeForMeet;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Meet1Teleop", group="TeleOp")
public class Meet1Teleop extends OpMode {

    // Drive motors
    private DcMotor rightBack;
    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor leftBack;

    // Scoring motors
    private DcMotorEx Shooter;
    private DcMotorEx Shooter2;

    private DcMotor Intake;
    private DcMotor Control;

    // Servos
    private CRServo LeftServo;
    private CRServo RightServo;

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

        // Set servo directions - adjust if servos move in opposite directions
        LeftServo.setDirection(CRServo.Direction.FORWARD);
        RightServo.setDirection(CRServo.Direction.REVERSE);

        // Set motor directions (adjust these based on your robot's configuration)
        Shooter.setDirection(DcMotorSimple.Direction.REVERSE);
//        Shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        // pidf sutff
        PIDFCoefficients SHOOTERpidfCoefficients = new PIDFCoefficients(16,0,0,15.5);
        Shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,SHOOTERpidfCoefficients);
        Shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,SHOOTERpidfCoefficients);

        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
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

        // GAMEPAD 2 - SCORING CONTROLS
        // Constantly running motors
        Shooter.setVelocity(1125);
        Shooter2.setVelocity(1125);
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
            Control.setPower(0.7);

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
        telemetry.addData("Drive Power", "LF: %.2f, RF: %.2f, LB: %.2f, RB: %.2f",
                leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        telemetry.addData("Servos Active", gamepad2.a);
        telemetry.addData("Trigger Timer", "%.2f seconds", triggerTimer.seconds());
        telemetry.addData("Control Position", Control.getCurrentPosition());
        telemetry.addData("Control Target", controlTargetPosition);
        telemetry.update();
    }
}
