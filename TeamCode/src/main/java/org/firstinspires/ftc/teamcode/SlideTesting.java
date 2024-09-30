package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp (name= "SlideTesting")
//@Disabled
public class SlideTesting extends OpMode {
    // motors
    DcMotor torque_slide;
    DcMotor speed_slide;

    @Override
    public void init() {
        torque_slide = hardwareMap.get(DcMotor.class, "torque_motor");
        speed_slide = hardwareMap.get(DcMotor.class, "speed_motor");

        torque_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        speed_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void loop() {
        //testing a change for github
        // encoders

        // wheel movement
        double left_x = gamepad1.left_stick_x;
        double left_y = gamepad1.left_stick_y;
        double joystick_turn = gamepad1.right_stick_x;
        double joystick_direction = Math.atan2(left_y, left_x);
        double joystick_magnitude = Math.sqrt((left_x * left_x) + (left_y * left_y));

        // actual wheel movement (for real) (100% working) (2024) (NOT CLICKBAIT) (u sure bout that?)

        // slide movement
        torque_slide.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        speed_slide.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
    }
}