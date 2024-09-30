package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp (name= "MecanumTeleOp")
//@Disabled
public class MecanumTeleOp extends OpMode {
    // motors
    DcMotor fl_Wheel;
    DcMotor bl_Wheel;
    DcMotor fr_Wheel;
    DcMotor br_Wheel;

    @Override
    public void init() {
        fl_Wheel = hardwareMap.get(DcMotor.class, "fl_motor");
        bl_Wheel = hardwareMap.get(DcMotor.class, "bl_motor");
        fr_Wheel = hardwareMap.get(DcMotor.class, "fr_motor");
        br_Wheel = hardwareMap.get(DcMotor.class, "br_motor");

        fr_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void loop() {
        //testing a change for github
        // encoders
        double fl_Position = fl_Wheel.getCurrentPosition();
        double bl_Position = bl_Wheel.getCurrentPosition();
        double fr_Position = fr_Wheel.getCurrentPosition();
        double br_Position = br_Wheel.getCurrentPosition();

        telemetry.addData("Front Left Wheel Pos", fl_Position);
        telemetry.addData("Back Left Wheel Pos", bl_Position);
        telemetry.addData("Front Right Wheel Pos", fr_Position);
        telemetry.addData("Back Right Wheel Pos", br_Position);
        telemetry.update();

        // wheel movement
        double left_x = gamepad1.left_stick_x;
        double left_y = gamepad1.left_stick_y;
        double joystick_turn = gamepad1.right_stick_x;
        double joystick_direction = Math.atan2(left_y, left_x);
        double joystick_magnitude = Math.sqrt((left_x * left_x) + (left_y * left_y));

        // actual wheel movement (for real) (100% working) (2024) (NOT CLICKBAIT) (u sure bout that?)
        fr_Wheel.setPower(-1 * Math.sin(joystick_direction - (0.25 * Math.PI)) * joystick_magnitude + joystick_turn);
        fl_Wheel.setPower(Math.sin(joystick_direction + (0.25 * Math.PI)) * joystick_magnitude + joystick_turn);
        br_Wheel.setPower(-1 * Math.sin(joystick_direction + (0.25 * Math.PI)) * joystick_magnitude + joystick_turn);
        bl_Wheel.setPower(Math.sin(joystick_direction - (0.25 * Math.PI)) * joystick_magnitude + joystick_turn);

        fr_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}

//blah