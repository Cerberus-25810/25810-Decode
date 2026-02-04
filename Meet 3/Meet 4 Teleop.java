package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.TreeMap;

@Config
@TeleOp(name = "Teleop", group = "Competition")
public class Teleop extends OpMode {

    // ===== FIELD POSITIONS =====
    private final Pose startPose =
            new Pose(21.792, 124.3442, Math.toRadians(143));

    private static final double GOAL_X = 10;
    private static final double GOAL_Y = 137;

    // Small residual calibration offset (0.83 deg) on top of the axis convention.
    // Derived from: 143 deg (known) - atan2(-dx, -dy) at startPose.
    private static final double HEADING_OFFSET = Math.toRadians(-0.8343);

    // Shooter offset from robot center (in robot frame)
    private static final double SHOOTER_OFFSET_X = 0.0;
    private static final double SHOOTER_OFFSET_Y = 0.0;

    // ===== AUTO-AIM TUNING =====
    private static final double AIM_KP = 0.9;
    private static final double AIM_KD = 0.1;

    boolean autoAimEnabled = false;
    boolean lastY = false;
    double lastHeadingError = 0;

    // ===== SHOOTER PID / FEEDFORWARD =====
    private static final double kP = 0.005, kI = 0.0, kD = 0.0, kF = 0.0;
    private static final double kV = 0.00035, kA = 0.0, kS = 0.065;

    // ===== HARDWARE =====
    private DcMotorEx Shooter, Shooter2;
    private DcMotor Intake, Control;
    private CRServo LeftServo, RightServo;
    private ServoImplEx Hood, Hood2;
    private claudepid pidController;

    // ===== INTERPOLATION TABLES =====
    // Format: (distance, velocity) and (distance, hoodPosition)
    private static final TreeMap<Double, Double> velocityTable = new TreeMap<>();
    private static final TreeMap<Double, Double> hoodTable = new TreeMap<>();

    static {
        // Data format: (Distance, Velocity, Hood Position)
        addPoint(30.45, 1080, 0.00);
        addPoint(35.21, 1195, 0.13);
        addPoint(40.27, 1265, 0.27);
        addPoint(45.14, 1330, 0.32);
        addPoint(50.31, 1390, 0.34);
        addPoint(55.14, 1435, 0.36);
        addPoint(60.04, 1530, 0.37);
        addPoint(62.09, 1620, 0.38);
        addPoint(71.08, 1620, 0.43);
        addPoint(81.84, 1690, 0.430);
        addPoint(85.94, 1785, 0.430);
    }


    private static void addPoint(double dist, double vel, double hood) {
        velocityTable.put(dist, vel);
        hoodTable.put(dist, hood);
    }

    // ===== INIT =====

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        initializeHardware();
        configureMotors();
        configureServos();

        pidController = new claudepid(kP, kI, kD, kF);
        pidController.setFeedforward(kV, kA, kS);

        follower.update();
    }

    private void initializeHardware() {
        Shooter = hardwareMap.get(DcMotorEx.class, "Leftshooter");
        Shooter2 = hardwareMap.get(DcMotorEx.class, "Rightshooter");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Control = hardwareMap.get(DcMotor.class, "Control");
        LeftServo = hardwareMap.get(CRServo.class, "LeftServo");
        RightServo = hardwareMap.get(CRServo.class, "RightServo");
        Hood = hardwareMap.get(ServoImplEx.class, "Hood");
        Hood2 = hardwareMap.get(ServoImplEx.class, "Hood2");
    }

    private void configureMotors() {
        Shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        Shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        Control.setDirection(DcMotorSimple.Direction.REVERSE);
        RightServo.setDirection(DcMotorSimple.Direction.REVERSE);

        // Use RUN_USING_ENCODER so getVelocity() works for the PID loop
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void configureServos() {
        Hood.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Hood2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Hood.setDirection(Servo.Direction.REVERSE);
        LeftServo.setDirection(CRServo.Direction.FORWARD);
        RightServo.setDirection(CRServo.Direction.REVERSE);
    }

    // ===== START =====

    @Override
    public void start() {
        follower.startTeleopDrive();
        Control.setPower(0);
        Intake.setPower(0.1);
        LeftServo.setPower(-1);
        RightServo.setPower(-1);
        lastHeadingError = 0;
    }

    // ===== MAIN LOOP =====

    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.getPose();

        // ---------- Auto-Aim Toggle (Y button) ----------
        boolean currentY = gamepad1.y;
        if (currentY && !lastY) {
            autoAimEnabled = !autoAimEnabled;
            if (autoAimEnabled) {
                lastHeadingError = 0;
            }
        }
        lastY = currentY;

        // ---------- Drivetrain ----------
        double drive = -gamepad1.left_stick_y * 0.8;
        double strafe = -gamepad1.left_stick_x * 0.8;
        double turn;

        if (autoAimEnabled) {
            double targetHeading = computeTargetHeading();
            double currentHeading = pose.getHeading();

            double headingError = MathFunctions.getTurnDirection(currentHeading, targetHeading)
                    * MathFunctions.getSmallestAngleDifference(currentHeading, targetHeading);

            double derivative = headingError - lastHeadingError;
            turn = AIM_KP * headingError + AIM_KD * derivative;
            lastHeadingError = headingError;

            turn = Math.max(-1.0, Math.min(1.0, turn));
        } else {
            turn = -gamepad1.right_stick_x * 0.8;
        }

        follower.setTeleOpDrive(drive, strafe, turn, true);

        // ---------- Shooter Speed + Hood (distance-based) ----------
        double distanceToGoal = Math.hypot(GOAL_X - pose.getX(), GOAL_Y - pose.getY());

        double targetVelocity = interpolate(velocityTable, distanceToGoal);
        double targetHoodPos = interpolate(hoodTable, distanceToGoal);

        Hood.setPosition(targetHoodPos);
        Hood2.setPosition(targetHoodPos);

        double currentVel = Shooter.getVelocity();
        double shooterPower = pidController.calculate(targetVelocity - currentVel, targetVelocity, 0);

        Shooter.setPower(Range.clip(shooterPower, -1, 1));
        Shooter2.setPower(Range.clip(shooterPower, -1, 1));

        // ---------- Intake / Control ----------
        if (gamepad1.right_trigger > 0.02) {
            Control.setPower(1);
            Intake.setPower(1);
            RightServo.setPower(1);
            LeftServo.setPower(1);
        } else {
            Control.setPower(0.7);
            Intake.setPower(0.9);
            RightServo.setPower(-1);
            LeftServo.setPower(-1);
        }

        // ---------- Telemetry ----------
        double targetHeading = computeTargetHeading();
        double errorDeg = Math.toDegrees(
                MathFunctions.getTurnDirection(pose.getHeading(), targetHeading)
                        * MathFunctions.getSmallestAngleDifference(pose.getHeading(), targetHeading)
        );

        telemetry.addData("Auto-Aim", autoAimEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("", "");
        telemetry.addData("Robot X", "%.2f", pose.getX());
        telemetry.addData("Robot Y", "%.2f", pose.getY());
        telemetry.addData("Robot Heading", "%.1f deg", Math.toDegrees(pose.getHeading()));
        telemetry.addData("", "");
        telemetry.addData("Target Heading", "%.1f deg", Math.toDegrees(targetHeading));
        telemetry.addData("Heading Error", "%.1f deg", errorDeg);
        telemetry.addData("Turn Output", "%.3f", turn);
        telemetry.addData("", "");
        telemetry.addData("Distance", "%.2f", distanceToGoal);
        telemetry.addData("Target Vel", "%.0f", targetVelocity);
        telemetry.addData("Current Vel", "%.0f", currentVel);
        telemetry.addData("Hood Pos", "%.2f", targetHoodPos);
        telemetry.update();
    }

    // ===== HELPER: Compute target heading for auto-aim =====

    /**
     * Compute the heading the robot needs to face to point at the goal.
     *
     * Pedro's heading convention has flipped axes relative to standard atan2,
     * so we use atan2(-dx, -dy) instead of atan2(dy, dx). A tiny residual
     * offset of -0.83 deg is added for calibration precision.
     */
    private double computeTargetHeading() {
        Pose robotPose = follower.getPose();

        double shooterX = robotPose.getX();
        double shooterY = robotPose.getY();

        if (SHOOTER_OFFSET_X != 0 || SHOOTER_OFFSET_Y != 0) {
            double cos = Math.cos(robotPose.getHeading());
            double sin = Math.sin(robotPose.getHeading());
            shooterX += SHOOTER_OFFSET_X * cos - SHOOTER_OFFSET_Y * sin;
            shooterY += SHOOTER_OFFSET_X * sin + SHOOTER_OFFSET_Y * cos;
        }

        double dx = GOAL_X - shooterX;
        double dy = GOAL_Y - shooterY;

        // atan2(-dx, -dy) matches Pedro's heading convention (verified: gives 143 at startPose)
        return Math.atan2(-dx, -dy) + HEADING_OFFSET;
    }

    // ===== HELPER: Linear interpolation over a TreeMap =====

    /**
     * Performs linear interpolation between data points in a TreeMap.
     * Clamps to the nearest endpoint if the input is out of range.
     */
    private double interpolate(TreeMap<Double, Double> table, double x) {
        if (table.isEmpty()) return 0;

        // Handle out-of-bounds (Clamp to nearest known point)
        if (x <= table.firstKey()) return table.get(table.firstKey());
        if (x >= table.lastKey()) return table.get(table.lastKey());

        // Get the two points surrounding 'x'
        Double lowKey = table.floorKey(x);
        Double highKey = table.ceilingKey(x);

        if (lowKey == null || highKey == null || lowKey.equals(highKey)) {
            return table.get(lowKey != null ? lowKey : highKey);
        }

        double lowVal = table.get(lowKey);
        double highVal = table.get(highKey);

        // Linear interpolation formula: y = y0 + (x - x0) * (y1 - y0) / (x1 - x0)
        return lowVal + (x - lowKey) * (highVal - lowVal) / (highKey - lowKey);
    }
}
