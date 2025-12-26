package org.firstinspires.ftc.teamcode.Meet1.FinalCodeForMeet;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Meet1.mechanisms.FlywheelLogic;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BroganAutoFinalBlue", group = "Autonomous")
public class BroganAutoFinalBlue extends OpMode {


    // ----------- FLYWHEEL SETUP --------------//

    private FlywheelLogic shooter = new FlywheelLogic();

    private ElapsedTime stateTimer = new ElapsedTime();

    private boolean shotsTriggered = false;

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private DcMotor Shooter;
    private DcMotor Intake;
    private DcMotor Control;

    // Servos
    private CRServo LeftServo;
    private CRServo RightServo;

    public enum PathState {
        // START POSITION > END POSITION
        // DRIVE (MOVEMENT) STATE
        // SHOOT (ATTEMPT TO SCORE) STATE

        DRIVE_STARTPOS_TO_SHOOTPOS,
        SHOOT_PRELOAD,
        //        DRIVE_SHOOTPOS_ENDPOS //TESTING POSE
        DRIVE_SHOOTPOS_TO_PICKUPLINEUP,
        DRIVE_PICKUPLINEUP_TO_PICKUP,
        DRIVE_PICKUPPOS_SHOOTPOS,
        SHOOT_PRELOAD_2
    }

    PathState pathState;


    // POSITION DECLARING
    private final Pose startPose = new Pose(21.792284866468844,124.34421364985162,Math.toRadians(143));
    private final Pose shootPose = new Pose(51,91, Math.toRadians(136));

    private final Pose endPose = new Pose(64,105,Math.toRadians(90)); //TESTING POSE

    private final Pose pickUpLineUp = new Pose(47.5,83.8, Math.toRadians(180));
    private final Pose pickUp = new Pose(23,81,Math.toRadians(180));

    private PathChain drive_StartPos_ShootPos,drive_ShootPos_EndPos,drive_ShootPos_PickUpLineUpPos,drive_PickUpLineUpPos_PickUP,drive_PickUp_Shoot;

    public void buildPaths(){
        // Put in coordinates for starting pose to ending pose
        drive_StartPos_ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

//        drive_ShootPos_EndPos = follower.pathBuilder()
//                .addPath(new BezierLine(shootPose,endPose))
//                .setLinearHeadingInterpolation(shootPose.getHeading(),endPose.getHeading())
//                .build();

        drive_ShootPos_PickUpLineUpPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,pickUpLineUp))
                .setLinearHeadingInterpolation(shootPose.getHeading(),endPose.getHeading())
                .build();

        drive_PickUpLineUpPos_PickUP = follower.pathBuilder()
                .addPath(new BezierLine(pickUpLineUp,pickUp))
                .setLinearHeadingInterpolation(pickUpLineUp.getHeading(),pickUp.getHeading())
                .build();

        drive_PickUp_Shoot = follower.pathBuilder()
                .addPath(new BezierLine(pickUp,shootPose))
                .setLinearHeadingInterpolation(pickUp.getHeading(),shootPose.getHeading())
                .build();


    }

    public void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_TO_SHOOTPOS:
                follower.setMaxPower(1);
                follower.followPath(drive_StartPos_ShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);//it will reset timer & switch state through func on line 81
                break;
            case SHOOT_PRELOAD:
                //check is follower done it's path and *3 sexconds has elapsed*
                if (!follower.isBusy()){
                    //TODO: add shooting logic stuff liek rampup things and servo controls

                    if (!follower.isBusy()){
                        // requested shots yet?
                        if (!shotsTriggered){
                            shooter.fireShots(4);
                            shotsTriggered = true;
                        }
                        else if(shotsTriggered && !shooter.isBusy()){
                            //shots are done free to trasistion
                            setPathState(PathState.DRIVE_SHOOTPOS_TO_PICKUPLINEUP);
                        }
                    }
                }
                break;

            case DRIVE_SHOOTPOS_TO_PICKUPLINEUP:
                follower.followPath(drive_ShootPos_PickUpLineUpPos,true);
                setPathState(PathState.DRIVE_PICKUPLINEUP_TO_PICKUP);
                stateTimer.reset();
                break;
            case DRIVE_PICKUPLINEUP_TO_PICKUP:
                if(!follower.isBusy()) {
                    follower.followPath(drive_PickUpLineUpPos_PickUP,true);
                    setPathState(PathState.DRIVE_PICKUPPOS_SHOOTPOS);
                    stateTimer.reset();
                }
                break;
            case DRIVE_PICKUPPOS_SHOOTPOS:
                if(!follower.isBusy()){
                    follower.followPath(drive_PickUp_Shoot);
                    telemetry.addLine("last path done");
                    stateTimer.reset();
                    setPathState(PathState.SHOOT_PRELOAD_2);
                }
                break;
            case SHOOT_PRELOAD_2:
                if (!follower.isBusy() && stateTimer.seconds() > 3.5){
                    //TODO: add shooting logic stuff liek rampup things and servo controls

                    if (!follower.isBusy()){
                        // requested shots yet?
                        if (!shotsTriggered){
                            shooter.fireShots(3);
                            shotsTriggered = true;
                        }
                        else if(shotsTriggered && !shooter.isBusy()){
                            //shots are done free to trasistion
                            setPathState(PathState.DRIVE_SHOOTPOS_TO_PICKUPLINEUP);
                        }
                    }
                }
                break;




            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();

        shotsTriggered = false;
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_TO_SHOOTPOS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        //opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        // TODO add in any other init mechanisims like flyweel and servos and ctrl motors
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
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
        shooter.init(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
    }

    public void start(){
        try {
            Thread.sleep(400); // 400ms delay
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        opModeTimer.resetTimer();
        setPathState(pathState);
        Shooter.setPower(0.61);
        Control.setPower(0.3);
        Intake.setPower(1);
        LeftServo.setPower(-1);
        RightServo.setPower(-1);
    }

    @Override
    public void loop(){
        follower.update();
        shooter.update();
        statePathUpdate();


        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y",follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());

    }

}
