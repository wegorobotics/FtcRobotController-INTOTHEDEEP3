package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "BlueRight2+4")
@Config
public class BlueRight24 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BR24 aut = new BR24(this, false);
        aut.waitForStart();
        while (!isStopRequested() && opModeIsActive()&& aut.isAutDone()) {
            aut.purp();
            aut.pre();
            aut.cycleIntake(5);
            aut.cycleDrop();
            aut.cycleIntake(3);
            aut.cycleDrop();
            aut.park();
            aut.update();
        }
    }
}