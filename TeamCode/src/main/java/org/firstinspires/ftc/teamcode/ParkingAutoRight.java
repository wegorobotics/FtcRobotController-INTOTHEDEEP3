package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name= "ParkingAutoRight")

public class ParkingAutoRight extends LinearOpMode {
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

        telemetry.setMsTransmissionInterval(11);
        //limelight.pipelineSwitch(0);
        //limelight.start();

        waitForStart();
        arm_servo.setPosition(0.64);
        claw_servo.setPosition(1);

        // move(1,0,-0.02,2000);

        // 0: right
        // 90: forward
        // -90: backward
        // 180: left

        move(1,90,0.01,3300); // direction originally 46, turn originally 0

        move(1, 180, 0.012, 1700); // direction originally 0, turn originally -0.01
        move(1, -90, 0, 3000); // direction originally 90, time originally 2500


        move(1, 90, -0.012, 1300); // direction originally 0, time originally 900
        move(1,0,0,900); // direction originally -90, time originally 2600
        move(1, 90, -0.012, 1300); // remove this line if reverting code
        /*
        move(1, 90, 0, 2400);
        move(1, 0, -0.012, 700);
        move(1,-90,0,2600);

         */



        /*
        move(1,90,0,500);
        move(0,0,0,10000);
        move(1,-90,0,500);
         */



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

        sleep(100);
    }
}
