package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp (name= "MecanumTeleOpTesting")
//@Disabled
public class MecanumTeleOpTesting extends OpMode {
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
    Servo claw_servo;

    private ElapsedTime runtime = new ElapsedTime();

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
        arm_servo.setPosition(0.64);
        claw_servo.setPosition(1);


    }

    double placing_Position = 0;
    double previous_position = 0;
    double previous_time = 0;


    public void loop() {
        //testing a change for github
        // encoders
        double fl_Position = fl_Wheel.getCurrentPosition();
        double bl_Position = bl_Wheel.getCurrentPosition();
        double fr_Position = fr_Wheel.getCurrentPosition();
        double br_Position = br_Wheel.getCurrentPosition();
        double placing_Position = placing_slide.getCurrentPosition();
        double arm_Position = arm_servo.getPosition();
        double claw_Position = claw_servo.getPosition();

        telemetry.addData("Front Left Wheel Pos", fl_Position);
        telemetry.addData("Back Left Wheel Pos", bl_Position);
        telemetry.addData("Front Right Wheel Pos", fr_Position);
        telemetry.addData("Back Right Wheel Pos", br_Position);
        //telemetry.addData("Placing Slide Pos", placing_Position);
        //telemetry.addData("Magnetic Limit Boolean", magnetic_limit.isPressed());
        telemetry.addData("Arm Pos", arm_Position);
        telemetry.addData("Claw Pos", claw_Position);

        // wheel movement
        double left_x = gamepad1.left_stick_x;
        double left_y = gamepad1.left_stick_y;
        double joystick_turn = gamepad1.right_stick_x;
        double joystick_direction = -1 * Math.atan2(left_y, left_x);
        double joystick_magnitude = Math.sqrt((left_x * left_x) + (left_y * left_y));

        double left_x2 = gamepad2.left_stick_x / 2;
        double left_y2 = gamepad2.left_stick_y / 2;
        double joystick_turn2 = gamepad2.right_stick_x / 2;
        double joystick_direction2 = -1 * Math.atan2(left_y2, left_x2) / 2;
        double joystick_magnitude2 = Math.sqrt((left_x2 * left_x2) + (left_y2 * left_y2)) / 2;

        fr_Wheel.setPower((-1 * Math.sin((joystick_direction + joystick_direction2) - (0.25 * Math.PI)) * (joystick_magnitude + joystick_magnitude2) + (joystick_turn + joystick_turn2)) / 2);
        br_Wheel.setPower((1 * Math.sin((joystick_direction + joystick_direction2) + (0.25 * Math.PI)) * (joystick_magnitude + joystick_magnitude2) - (joystick_turn + joystick_turn2)) / 2);
        fl_Wheel.setPower((-1 * Math.sin((joystick_direction + joystick_direction2) + (0.25 * Math.PI)) * (joystick_magnitude + joystick_magnitude2) - (joystick_turn + joystick_turn2)) / 2);
        bl_Wheel.setPower((1 * Math.sin((joystick_direction + joystick_direction2) - (0.25 * Math.PI)) * (joystick_magnitude + joystick_magnitude2) + (joystick_turn + joystick_turn2)) / 2);

        fr_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // slide movement
        double right;
        double left;
        double right2;
        double left2;

        right = gamepad1.right_trigger;
        left = gamepad1.left_trigger;
        right2 = gamepad2.right_trigger;
        left2 = gamepad2.left_trigger;

        //placing_slide.setPower(right + right2 - left - left2);
        //telemetry.addData("Placing Slide Pos", placing_Position);

        //linear slide P in PID?
        double commanded_speed = right + right2 - left - left2;
        if (commanded_speed > 1) {
            commanded_speed = 1;
        } else if (commanded_speed < -1) {
            commanded_speed = -1;
        }

        double current_position = placing_Position;
        double current_time = runtime.seconds();

        double change_in_position = current_position - previous_position;
        double change_in_time = current_time - previous_time;

        double actual_speed = change_in_position / change_in_time;
        double speed_error = commanded_speed - actual_speed;
        double error_constant = 0.3;

        placing_slide.setPower(error_constant * speed_error);

        previous_position = current_position;
        previous_time = current_time;


        // arm movement
        final double arm_speed = 0.002;
        if (gamepad2.dpad_down) {
            arm_Position += arm_speed;
        } else if (gamepad2.dpad_up) {
            arm_Position -= arm_speed;
        }
        arm_servo.setPosition(arm_Position);

        // claw movement
        final double claw_speed = 0.003;
        if (gamepad2.x) {
            claw_Position += claw_speed;
        } else if (gamepad2.a) {
            claw_Position -= claw_speed;
        }
        claw_servo.setPosition(claw_Position);

        telemetry.update();
    }
}

//bleh