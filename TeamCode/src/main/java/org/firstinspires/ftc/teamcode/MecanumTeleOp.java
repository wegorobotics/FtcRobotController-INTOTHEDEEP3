package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp (name= "MecanumTeleOp")
//@Disabled
public class MecanumTeleOp extends OpMode {
    // motors
    DcMotor fl_Wheel;
    DcMotor bl_Wheel;
    DcMotor fr_Wheel;
    DcMotor br_Wheel;
    DcMotor placing_slide;
    DcMotor climbing_slide;
    //private Limelight3A limelight;
    Servo arm_servo;
    //TouchSensor magnetic_limit;

    @Override
    public void init() {
        fl_Wheel = hardwareMap.get(DcMotor.class, "fl_motor");
        bl_Wheel = hardwareMap.get(DcMotor.class, "bl_motor");
        fr_Wheel = hardwareMap.get(DcMotor.class, "fr_motor");
        br_Wheel = hardwareMap.get(DcMotor.class, "br_motor");
        placing_slide = hardwareMap.get(DcMotor.class, "placing_motor");
        climbing_slide = hardwareMap.get(DcMotor.class, "climbing_motor");
        //limelight = hardwareMap.get(Limelight3A.class, "Limelight 3A");
        arm_servo = hardwareMap.get(Servo.class, "arm");
        //magnetic_limit = hardwareMap.get(TouchSensor.class, "magnetic_limit");

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
    }

    public void loop() {
        //testing a change for github
        // encoders
        double fl_Position = fl_Wheel.getCurrentPosition();
        double bl_Position = bl_Wheel.getCurrentPosition();
        double fr_Position = fr_Wheel.getCurrentPosition();
        double br_Position = br_Wheel.getCurrentPosition();
        double arm_Position = arm_servo.getPosition();

        telemetry.addData("Front Left Wheel Pos", fl_Position);
        telemetry.addData("Back Left Wheel Pos", bl_Position);
        telemetry.addData("Front Right Wheel Pos", fr_Position);
        telemetry.addData("Back Right Wheel Pos", br_Position);
        //telemetry.addData("Magnetic Limit Boolean", magnetic_limit.isPressed());

        /*
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            telemetry.addData("April Tag X Pos", result.getTx());
            telemetry.addData("April Tag Y Pos", result.getTy());
            telemetry.addData("April Tag Area %", result.getTa());
            telemetry.addData("Limelight Pipeline", result.getPipelineIndex());
        }
        */

        //pipeline 0 = chamber side
        //pipeline 1 = basket side
        //pipeline 2 = observation side

        telemetry.update();

        // wheel movement
        double left_x = gamepad1.left_stick_x;
        double left_y = gamepad1.left_stick_y;
        double joystick_turn = gamepad1.right_stick_x;
        double joystick_direction = -1 * Math.atan2(left_y, left_x);
        double joystick_magnitude = Math.sqrt((left_x * left_x) + (left_y * left_y));

        // setting power of wheels based on joystick data
        /*
        fr_Wheel.setPower((-1 * Math.sin(joystick_direction + (0.25 * Math.PI)) * joystick_magnitude + joystick_turn) / 2);
        fl_Wheel.setPower((1 * Math.sin(joystick_direction + (0.25 * Math.PI)) * joystick_magnitude + joystick_turn) / 2);
        br_Wheel.setPower((-1 * Math.sin(joystick_direction - (0.25 * Math.PI)) * joystick_magnitude + joystick_turn) / 2);
        bl_Wheel.setPower((1 * Math.sin(joystick_direction - (0.25 * Math.PI)) * joystick_magnitude + joystick_turn) / 2);
        */

        fr_Wheel.setPower((-1 * Math.sin(joystick_direction - (0.25 * Math.PI)) * joystick_magnitude + joystick_turn) / 2);
        br_Wheel.setPower((1 * Math.sin(joystick_direction + (0.25 * Math.PI)) * joystick_magnitude - joystick_turn) / 2);
        fl_Wheel.setPower((-1 * Math.sin(joystick_direction + (0.25 * Math.PI)) * joystick_magnitude - joystick_turn) / 2);
        bl_Wheel.setPower((1 * Math.sin(joystick_direction - (0.25 * Math.PI)) * joystick_magnitude + joystick_turn) / 2);

        fr_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // slide movement

        placing_slide.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

        // arm movement
        final double arm_speed = 0.01;
        if (gamepad1.x) {
            arm_Position += arm_speed;
        } else if (gamepad1.y) {
            arm_Position -= arm_speed;
        }
        arm_servo.setPosition(arm_Position);
    }
}

//bleh