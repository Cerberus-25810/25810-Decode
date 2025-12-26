package org.firstinspires.ftc.teamcode.Meet1.mechanisms;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlyWheelPIDF extends OpMode {

    public DcMotorEx Shooter;

    public double highVelocity = 1120;
    public double lowVelocity = 800;

    public double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;
    double[] stepSizes = {10.0,1.0,0.1,0.001,0.0001};
    int stepIndex = 1;


    @Override
    public void init(){
        Shooter = hardwareMap.get(DcMotorEx.class,"Shooter");
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients SHOOTERpidfCoefficients = new PIDFCoefficients(P,0,0,F);
        Shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,SHOOTERpidfCoefficients);

        telemetry.addLine("init complete");

    }

    @Override
    public void loop(){

            if (gamepad1.yWasPressed()){
                if (curTargetVelocity == highVelocity){
                    curTargetVelocity = lowVelocity;
                } else { curTargetVelocity = highVelocity;}
            }

            if (gamepad1.bWasPressed()){
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }

            if (gamepad1.dpadLeftWasPressed()){
                F-= stepSizes[stepIndex];
            }
            if (gamepad1.dpadRightWasPressed()){
                F+= stepSizes[stepIndex];
            }


            if (gamepad1.dpadDownWasPressed()){
                P+= stepSizes[stepIndex];
            }
            if (gamepad1.dpadUpWasPressed()){
                P-= stepSizes[stepIndex];
            }


            PIDFCoefficients SHOOTERpidfCoefficients = new PIDFCoefficients(P,0,0,F);
            Shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,SHOOTERpidfCoefficients);

            Shooter.setVelocity(curTargetVelocity);

            double curVelocity = Shooter.getVelocity();
            double error = curTargetVelocity - curVelocity;

           telemetry.addData("Target Velocity", curTargetVelocity);
           telemetry.addData("Current Velocity", "%.2f",curVelocity);
           telemetry.addData("eror","%.2f",error);
           telemetry.addLine("--------------------------------");
           telemetry.addData("Tuning P","%.4f (D-PAD U/D)",P);
           telemetry.addData("Tuning F","%.4f (D-Pad L/R)",F);
           telemetry.addData("Step Size","%.4f (B Button",stepSizes[stepIndex]);



    }
}
