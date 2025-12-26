package org.firstinspires.ftc.teamcode.Meet1.mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FlywheelLogic {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private ElapsedTime stateTimer = new ElapsedTime();

    private DcMotorEx Shooter;
    private DcMotor Intake;
    private DcMotor Control;

    // Servos
    private CRServo LeftServo;
    private CRServo RightServo;

    private enum FlywheelState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        RESET
    }

    private FlywheelState flywheelState;

    // ------------------ Servo Constants ---------------- //

    private double ServoRunTime = 0.6;    // Time servos run to push ball
    private double ServoStopTime = 0.35;   // Time servos stop between shots

    // ---------------- Flywheel Constants -------------//

    private int shotsRemaining = 0;
    private double MIN_FLYWHEEL_RPM = 1150;
    private double TARGET_FLYWHEEL_RPM = 1150;

    private double FLYWHEEL_MAX_SPINUP_TIME = 0.3;

    public void init(HardwareMap hardwareMap){
        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Control = hardwareMap.get(DcMotor.class, "Control");

        // Set Control motor to use encoder for position control
        Control.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Control.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize servos
        LeftServo = hardwareMap.get(CRServo.class, "LeftServo");
        RightServo = hardwareMap.get(CRServo.class, "RightServo");

        // Set servo directions - adjust if servos move in opposite directions
        LeftServo.setDirection(CRServo.Direction.FORWARD);
        RightServo.setDirection(CRServo.Direction.REVERSE);

        // Set motor directions (adjust these based on your robot's configuration)
        Shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelState = FlywheelState.IDLE;
        Shooter.setVelocity(0);

        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients SHOOTERpidfCoefficients = new PIDFCoefficients(16,0,0,15.5);
        Shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,SHOOTERpidfCoefficients);
    }

    public void update(){
        switch (flywheelState){
            case IDLE:
                if (shotsRemaining > 0){
                    Shooter.setVelocity(TARGET_FLYWHEEL_RPM);

                    stateTimer.reset();
                    flywheelState = FlywheelState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                //Shooter.setPower(TARGET_FLYWHEEL_RPM);
                if (TARGET_FLYWHEEL_RPM > MIN_FLYWHEEL_RPM || stateTimer.seconds() > FLYWHEEL_MAX_SPINUP_TIME){
                    Control.setPower(0.3);
                    // START servos running to push ball
                    LeftServo.setPower(1);
                    RightServo.setPower(1);
                    stateTimer.reset();

                    flywheelState = FlywheelState.LAUNCH;
                }
                break;
            case LAUNCH:
                // Servos run for ServoRunTime, then we STOP them
                if (stateTimer.seconds() > ServoRunTime) {
                    shotsRemaining--; // Decrement shot count

                    // STOP the servos (but keep Control running)
                    LeftServo.setPower(-1);
                    RightServo.setPower(-1);

                    stateTimer.reset();
                    flywheelState = FlywheelState.RESET;
                }
                break;
            case RESET:
                // Servos are stopped, wait for ServoStopTime before next shot
                if (stateTimer.seconds() > ServoStopTime) {
                    if (shotsRemaining > 0){
                        // More shots to fire - go back to SPIN_UP which will start servos again
                        stateTimer.reset();
                        flywheelState = FlywheelState.SPIN_UP;
                    }
                    else {
                        // All shots fired - stop everything
//                        Shooter.setPower(0);
//                        Control.setPower(0);
                        LeftServo.setPower(-1);
                        RightServo.setPower(-1);
                        flywheelState = FlywheelState.IDLE;
                        stateTimer.reset();
                    }
                }
                break;
        }

    }

    public void fireShots(int numberOfShots){
        if(flywheelState == FlywheelState.IDLE){
            shotsRemaining = numberOfShots;
        }
    }

    public boolean isBusy(){
        return flywheelState != FlywheelState.IDLE;
    }

}
