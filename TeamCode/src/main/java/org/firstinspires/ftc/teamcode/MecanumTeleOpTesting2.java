package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp (name= "MecanumTeleOpTesting2")
//@Disabled
public class MecanumTeleOpTesting2 extends OpMode {
    // motors
    DcMotor fl_Wheel;
    DcMotor bl_Wheel;
    DcMotor fr_Wheel;
    DcMotor br_Wheel;
    DcMotor right_slide;
    DcMotor left_slide;
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
        right_slide = hardwareMap.get(DcMotor.class, "placing_motor");
        left_slide = hardwareMap.get(DcMotor.class, "climbing_motor");
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
        right_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        fr_Wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl_Wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_Wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_Wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fr_Wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl_Wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br_Wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl_Wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.setMsTransmissionInterval(11);
        //limelight.pipelineSwitch(0);
        //limelight.start();
        arm_servo.setPosition(0.16);
        claw_servo.setPosition(0.66);


    }

    //no mans' land
    double placing_Position = 0;
    double previous_position = 0;
    double previous_time = 0;
    double integral_sum = 0;
    double previous_error = 0;
    double derivative = 0;
    double placing_Position2 = 0;
    double previous_position2 = 0;
    double previous_time2 = 0;
    double integral_sum2 = 0;
    double previous_error2 = 0;
    double derivative2 = 0;
    double commanded_position = 0;
    double commanded_position2 = 0;


    public void loop() {
        //testing a change for github
        // encoders
        double fl_Position = fl_Wheel.getCurrentPosition();
        double bl_Position = bl_Wheel.getCurrentPosition();
        double fr_Position = fr_Wheel.getCurrentPosition();
        double br_Position = br_Wheel.getCurrentPosition();
        double placing_Position = right_slide.getCurrentPosition();
        double placing_Position2 = left_slide.getCurrentPosition();
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


        //PID?
        double commanded_speed = right + right2 - left - left2;
        if (commanded_speed > 1) {
            commanded_speed = 1;
        } else if (commanded_speed < -1) {
            commanded_speed = -1;
        }

        commanded_position += right + right2 - left - left2;

        double current_position = placing_Position;
        double current_time = runtime.seconds();

        //actual velocity
        double change_in_position = current_position - previous_position;
        double change_in_time = current_time - previous_time;
        double actual_speed = (change_in_position / change_in_time) / 1950;

        double error_constant = 0.06;
        double integral_constant = 0;
        double derivative_constant = 0;

        //P
        //double current_error = commanded_speed - actual_speed;
        double current_error = commanded_position - current_position;

        //I
        integral_sum += (current_error * change_in_time);

        //D
        derivative = (current_error - previous_error) / change_in_time;

        //setting power
        double final_power = (error_constant * current_error) + (integral_constant * integral_sum) - (derivative_constant * derivative);
        right_slide.setPower(final_power);

        previous_position = current_position;
        previous_time = current_time;
        previous_error = current_error;

        //PID 2nd slide!!
        //PID?

        double current_position2 = placing_Position2;
        double current_time2 = runtime.seconds();

        //actual velocity
        double change_in_position2 = current_position2 - previous_position2;
        double change_in_time2 = current_time2 - previous_time2;
        double actual_speed2 = (change_in_position2 / change_in_time2) / 1950;

        //P
        //double current_error2 = (-1 * commanded_speed) - actual_speed2;
        double current_error2 = (-1 * commanded_position) - current_position2;

        //I
        integral_sum2 += (current_error2 * change_in_time2);

        //D
        derivative2 = (current_error2 - previous_error2) / change_in_time2;

        //setting power
        double final_power2 = (error_constant * current_error2) + (integral_constant * integral_sum2) - (derivative_constant * derivative2);
        //double power_constant = -1.1;
        //left_slide.setPower(final_power * power_constant);
        left_slide.setPower(final_power2);

        previous_position2 = current_position2;
        previous_time2 = current_time2;
        previous_error2 = current_error2;

        telemetry.addData("Final Power 1", final_power);
        telemetry.addData("Final Power 2", final_power2);
        telemetry.addData("current error", current_error);
        telemetry.addData("current error2", current_error2);
        telemetry.addData("integral sum", integral_sum);
        telemetry.addData("derivative", derivative);
        telemetry.addData("actual speed", actual_speed);
        telemetry.addData("actual speed2", actual_speed2);
        telemetry.addData("right slide position", right_slide.getCurrentPosition());
        telemetry.addData("left slide position", left_slide.getCurrentPosition());

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