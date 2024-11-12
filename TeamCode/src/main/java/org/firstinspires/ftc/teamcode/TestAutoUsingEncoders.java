package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name= "TestAutoUsingEncoders")

public class TestAutoUsingEncoders extends LinearOpMode {
    // motors
    DcMotor fl_Wheel;
    DcMotor bl_Wheel;
    DcMotor fr_Wheel;
    DcMotor br_Wheel;
    DcMotor placing_slide;
    DcMotor climbing_slide;
    //private Limelight3A limelight;
    Servo arm_servo;
    Servo claw_servo;

    @Override
    public void runOpMode(){
        fl_Wheel = hardwareMap.get(DcMotor.class, "fl_motor");
        bl_Wheel = hardwareMap.get(DcMotor.class, "bl_motor");
        fr_Wheel = hardwareMap.get(DcMotor.class, "fr_motor");
        br_Wheel = hardwareMap.get(DcMotor.class, "br_motor");
        placing_slide = hardwareMap.get(DcMotor.class, "placing_motor");
        climbing_slide = hardwareMap.get(DcMotor.class, "climbing_motor");
        //limelight = hardwareMap.get(Limelight3A.class, "Limelight 3A");
        arm_servo = hardwareMap.get(Servo.class, "arm");
        claw_servo = hardwareMap.get(Servo.class, "claw");

        fr_Wheel.setDirection(DcMotor.Direction.REVERSE);
        fl_Wheel.setDirection(DcMotor.Direction.FORWARD);
        br_Wheel.setDirection(DcMotor.Direction.REVERSE);
        bl_Wheel.setDirection(DcMotor.Direction.REVERSE);

        fr_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        placing_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climbing_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fr_Wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl_Wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br_Wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl_Wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        telemetry.setMsTransmissionInterval(11);
        //limelight.pipelineSwitch(0);
        //limelight.start();

        waitForStart();
        arm_servo.setPosition(0.64);
        claw_servo.setPosition(1);

        fr_Wheel.setTargetPosition(860);
        fl_Wheel.setTargetPosition(860);
        br_Wheel.setTargetPosition(860);
        bl_Wheel.setTargetPosition(860);

        fr_Wheel.setPower(-0.5);
        fl_Wheel.setPower(-0.5);
        br_Wheel.setPower(0.5);
        bl_Wheel.setPower(0.5);

        fr_Wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl_Wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br_Wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl_Wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    public void move(double magnitude, double direction, double turn, long time) {
        double radians = 1 * ((direction) / 180) * Math.PI;

        fr_Wheel.setPower((-1 * Math.sin(radians - (0.25 * Math.PI)) * magnitude + turn) / 2);
        br_Wheel.setPower((1 * Math.sin(radians + (0.25 * Math.PI)) * magnitude - turn) / 2);
        fl_Wheel.setPower((-1 * Math.sin(radians + (0.25 * Math.PI)) * magnitude - turn) / 2);
        bl_Wheel.setPower((1 * Math.sin(radians - (0.25 * Math.PI)) * magnitude + turn) / 2);

        sleep(time);

        fr_Wheel.setPower(0);
        bl_Wheel.setPower(0);
        fl_Wheel.setPower(0);
        br_Wheel.setPower(0);
    }
}
