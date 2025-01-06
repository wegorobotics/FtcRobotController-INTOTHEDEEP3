package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name= "ParkingAutoSTRAIGHT")

public class ParkingAutoSTRAIGHT extends LinearOpMode {
    // motors
    DcMotor fl_Wheel;
    DcMotor bl_Wheel;
    DcMotor fr_Wheel;
    DcMotor br_Wheel;
    DcMotor placing_slide;DcMotor climbing_slide;
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

        fr_Wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl_Wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_Wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_Wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fr_Wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl_Wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br_Wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl_Wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.setMsTransmissionInterval(11);
        //limelight.pipelineSwitch(0);
        //limelight.start();

        waitForStart();
        arm_servo.setPosition(0.11);
        claw_servo.setPosition(0.6);

        move(1, 0, 0, 4000);
        double fl_Position = fl_Wheel.getCurrentPosition();
        double bl_Position = bl_Wheel.getCurrentPosition();
        double fr_Position = fr_Wheel.getCurrentPosition();
        double br_Position = br_Wheel.getCurrentPosition();

        telemetry.addData("Front Left Wheel Pos", fl_Position);
        telemetry.addData("Back Left Wheel Pos", bl_Position);
        telemetry.addData("Front Right Wheel Pos", fr_Position);
        telemetry.addData("Back Right Wheel Pos", br_Position);

        telemetry.update();

        sleep(50000);

    }

    public void move(double magnitude, double direction, double turn, long time) {
        double radians = 1 * ((direction) / 180) * Math.PI;

        fr_Wheel.setPower(1.004 * (-1 * Math.sin(radians - (0.25 * Math.PI)) * magnitude + turn) / 2);
        br_Wheel.setPower(1.000 * (1 * Math.sin(radians + (0.25 * Math.PI)) * magnitude - turn) / 2);
        fl_Wheel.setPower(1.227 * (-1 * Math.sin(radians + (0.25 * Math.PI)) * magnitude - turn) / 2);
        bl_Wheel.setPower(1.088 * (1 * Math.sin(radians - (0.25 * Math.PI)) * magnitude + turn) / 2);

        sleep(time);


        fr_Wheel.setPower(0);
        bl_Wheel.setPower(0);
        fl_Wheel.setPower(0);
        br_Wheel.setPower(0);
    }
}
