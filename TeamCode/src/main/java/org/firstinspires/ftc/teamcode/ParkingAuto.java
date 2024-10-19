package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name= "ParkingAuto")

public class ParkingAuto extends LinearOpMode {
    // motors
    DcMotor fl_Wheel;
    DcMotor bl_Wheel;
    DcMotor fr_Wheel;
    DcMotor br_Wheel;
    DcMotor torque_slide;
    DcMotor speed_slide;
    //private Limelight3A limelight;
    Servo arm_servo;

    @Override
    public void runOpMode(){
        fl_Wheel = hardwareMap.get(DcMotor.class, "fl_motor");
        bl_Wheel = hardwareMap.get(DcMotor.class, "bl_motor");
        fr_Wheel = hardwareMap.get(DcMotor.class, "fr_motor");
        br_Wheel = hardwareMap.get(DcMotor.class, "br_motor");
        torque_slide = hardwareMap.get(DcMotor.class, "torque_motor");
        speed_slide = hardwareMap.get(DcMotor.class, "speed_motor");
        //limelight = hardwareMap.get(Limelight3A.class, "Limelight 3A");

        fr_Wheel.setDirection(DcMotor.Direction.FORWARD);
        fl_Wheel.setDirection(DcMotor.Direction.REVERSE);
        br_Wheel.setDirection(DcMotor.Direction.REVERSE);
        bl_Wheel.setDirection(DcMotor.Direction.REVERSE);

        fr_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        torque_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        speed_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.setMsTransmissionInterval(11);
        //limelight.pipelineSwitch(0);
        //limelight.start();

        waitForStart();

        move(1, 90, 0, 4000);





    }

    public void move(double magnitude, double direction, double turn, long time) {
        double radians = -1 * ((direction + 90) / 180) * Math.PI;

        fr_Wheel.setPower((Math.sin(radians + (0.25 * Math.PI)) * magnitude + turn) / 2);
        fl_Wheel.setPower((-1 * Math.sin(radians - (0.25 * Math.PI)) * magnitude + turn) / 2);
        br_Wheel.setPower((Math.sin(radians - (0.25 * Math.PI)) * magnitude + turn) / 2);
        bl_Wheel.setPower((-1 * Math.sin(radians + (0.25 * Math.PI)) * magnitude + turn) / 2);

        sleep(time);

        fr_Wheel.setPower(0);
        bl_Wheel.setPower(0);
        fl_Wheel.setPower(0);
        br_Wheel.setPower(0);
    }
}
