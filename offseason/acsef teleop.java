package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Meet4.Pidshootercontroller;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name = "TeleopACEF", group = "Competition")
public class TeleopSimpleACSEF extends OpMode {

    // ===== SHOOTER =====
    private static final double VELOCITY_LOW  = 1300;
    private static final double VELOCITY_HIGH = 10000;
    private double targetVelocity = VELOCITY_LOW;
    private boolean lastA = false;
    private boolean lastB = false;
    private static final double kP = 0.003, kI = 0.0, kD = 0.0, kF = 0.0;
    private static final double kV = 0.00035, kA = 0.0, kS = 0.065;

    // ===== HOOD =====
    private static final double HOOD_MIN  = 0.0;
    private static final double HOOD_MAX  = 0.5;
    private static final double HOOD_STEP = 0.002;
    private double hoodPosition = 0.25;

    // ===== HARDWARE =====
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx Shooter, Shooter2;
    private DcMotor Intake, Control;
    private CRServo LeftServo, RightServo;
    private ServoImplEx Hood, Hood2;
    private Pidshootercontroller pidController;

    @Override
    public void init() {
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Shooter    = hardwareMap.get(DcMotorEx.class,   "Leftshooter");
        Shooter2   = hardwareMap.get(DcMotorEx.class,   "Rightshooter");
        Intake     = hardwareMap.get(DcMotor.class,     "Intake");
        Control    = hardwareMap.get(DcMotor.class,     "Control");
        LeftServo  = hardwareMap.get(CRServo.class,     "LeftServo");
        RightServo = hardwareMap.get(CRServo.class,     "RightServo");
        Hood       = hardwareMap.get(ServoImplEx.class, "Hood");
        Hood2      = hardwareMap.get(ServoImplEx.class, "Hood2");

        Shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        Shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        Control.setDirection(DcMotorSimple.Direction.REVERSE);
        RightServo.setDirection(DcMotorSimple.Direction.REVERSE);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Hood.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Hood2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Hood.setDirection(Servo.Direction.FORWARD);
        Hood2.setDirection(Servo.Direction.REVERSE);
        LeftServo.setDirection(CRServo.Direction.FORWARD);
        RightServo.setDirection(CRServo.Direction.REVERSE);

        Hood.setPosition(hoodPosition);
        Hood2.setPosition(hoodPosition);

        pidController = new Pidshootercontroller(kP, kI, kD, kF);
        pidController.setFeedforward(kV, kA, kS);

        telemetry.addData("Status", "Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        Control.setPower(0.5);
        Intake.setPower(0.1);
        LeftServo.setPower(-1);
        RightServo.setPower(-1);
    }

    @Override
    public void loop() {

        // ===== VELOCITY TOGGLE =====
        if (gamepad1.a && !lastA) targetVelocity = VELOCITY_LOW;
        if (gamepad1.b && !lastB) targetVelocity = VELOCITY_HIGH;
        lastA = gamepad1.a;
        lastB = gamepad1.b;

        // ===== HOOD =====
        if (gamepad1.dpad_up)   hoodPosition = Math.min(HOOD_MAX, hoodPosition + HOOD_STEP);
        if (gamepad1.dpad_down) hoodPosition = Math.max(HOOD_MIN, hoodPosition - HOOD_STEP);
        Hood.setPosition(hoodPosition);
        Hood2.setPosition(hoodPosition);

        // ===== DRIVE =====
        double drive  = -gamepad1.left_stick_y * 0.5;
        double strafe =  gamepad1.left_stick_x * 0.5;
        double turn   =  gamepad1.right_stick_x * 0.5;

        double fl = drive + strafe + turn;
        double fr = drive - strafe - turn;
        double bl = drive - strafe + turn;
        double br = drive + strafe - turn;

        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));

        leftFront.setPower(fl / max);
        rightFront.setPower(fr / max);
        leftBack.setPower(bl / max);
        rightBack.setPower(br / max);

        // ===== SHOOTER =====
        double currentVel   = Shooter.getVelocity();
        double shooterPower = pidController.calculate(
                targetVelocity - currentVel, targetVelocity, 0);
        Shooter.setPower(Range.clip(shooterPower, -1, 1));
        Shooter2.setPower(Range.clip(shooterPower, -1, 1));

        // ===== INTAKE =====
        if (gamepad1.right_trigger > 0.02) {
            Control.setPower(1);
            Intake.setPower(1);
            RightServo.setPower(1);
            LeftServo.setPower(1);
        } else {
            Control.setPower(0.5);
            Intake.setPower(0.9);
            RightServo.setPower(-1);
            LeftServo.setPower(-1);
        }

        // ===== TELEMETRY =====
        telemetry.addData("Hood",    "%.3f", hoodPosition);
        telemetry.addData("Shooter", "target:%.0f  actual:%.0f  [A=1300 B=2000]", targetVelocity, currentVel);
        telemetry.update();
    }
}
