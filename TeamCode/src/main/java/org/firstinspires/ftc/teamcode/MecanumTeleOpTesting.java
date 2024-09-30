package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@TeleOp (name= "MecanumTeleOpTesting")
//@Disabled
public class MecanumTeleOpTesting extends OpMode {
    // motors
    DcMotor fl_Wheel;
    DcMotor bl_Wheel;
    DcMotor fr_Wheel;
    DcMotor br_Wheel;
    DcMotor torque_slide;
    DcMotor speed_slide;
    private Limelight3A limelight;

    @Override
    public void init() {
        fl_Wheel = hardwareMap.get(DcMotor.class, "fl_motor");
        bl_Wheel = hardwareMap.get(DcMotor.class, "bl_motor");
        fr_Wheel = hardwareMap.get(DcMotor.class, "fr_motor");
        br_Wheel = hardwareMap.get(DcMotor.class, "br_motor");
        torque_slide = hardwareMap.get(DcMotor.class, "torque_motor");
        speed_slide = hardwareMap.get(DcMotor.class, "speed_motor");
        limelight = hardwareMap.get(Limelight3A.class, "Limelight 3A");

        fr_Wheel.setDirection(DcMotor.Direction.REVERSE);
        fl_Wheel.setDirection(DcMotor.Direction.FORWARD);
        br_Wheel.setDirection(DcMotor.Direction.REVERSE);
        bl_Wheel.setDirection(DcMotor.Direction.FORWARD);

        fr_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        torque_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        speed_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
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

        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("tag_number", limelight.getLatestResult());
            }
        }

        // wheel movement
        double left_x = gamepad1.left_stick_x;
        double left_y = gamepad1.left_stick_y;
        double joystick_turn = gamepad1.right_stick_x;
        double joystick_direction = Math.atan2(left_y, left_x);
        double joystick_magnitude = Math.sqrt((left_x * left_x) + (left_y * left_y));

        // actual wheel movement (for real) (100% working) (2024) (NOT CLICKBAIT) (u sure bout that?)
        fr_Wheel.setPower((-1 * Math.sin(joystick_direction + (0.25 * Math.PI)) * joystick_magnitude + joystick_turn) / 2);
        fl_Wheel.setPower((Math.sin(joystick_direction + (0.25 * Math.PI)) * joystick_magnitude + joystick_turn) / 2);
        br_Wheel.setPower((-1 * Math.sin(joystick_direction - (0.25 * Math.PI)) * joystick_magnitude + joystick_turn) / 2);
        bl_Wheel.setPower((Math.sin(joystick_direction - (0.25 * Math.PI)) * joystick_magnitude + joystick_turn) / 2);

        fr_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // slide movement
        torque_slide.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        speed_slide.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
    }
}

//bleh