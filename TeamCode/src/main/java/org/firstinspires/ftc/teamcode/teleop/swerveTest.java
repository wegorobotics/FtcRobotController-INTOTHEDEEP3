package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teamUtil.gamepadEX.button;
import org.firstinspires.ftc.teamcode.teamUtil.robotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.robotConstants;

@TeleOp(name="Swerve Test", group="Worlds")
public class swerveTest extends OpMode{

    robotConfig r;
    private final ElapsedTime runTime = new ElapsedTime();
    Telemetry.Item item1;

    @Override
    public void init(){
        telemetry.addData("Status", "Initialising");
        telemetry.update();
        r = robotConfig.getInstance(this);
        r.initSystems(robotConstants.configuredSystems.BOTH_MODULES);
        telemetry.addData("Status", "Initialised");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        r.gamepadEX1.a.debounce(button.debouncingType.BOTH, 0.5); //normally the debouncing value should be more then 10x smaller than this, large for testing purposes
        runTime.reset();
        item1 = telemetry.addData("mbruh", false);
    }

    @Override
    public void loop(){
        r.commandControl.conditionalAction(() -> (r.gamepadEX1.a.isPressed() && r.gamepadEX2.a.onPress()), () -> item1.setValue(true));

        r.commandControl.conditionalAction(() -> (r.gamepadEX1.b.isPressed() && r.gamepadEX2.b.onPress()), () -> item1.setValue(false));

        r.encoderRead.encoderBulkRead();
        r.swerve.manualDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_stick_y, (1-gamepad1.right_trigger), false);
        r.systemsEndLoopUpdate();
    }

    @Override
    public void stop(){
        r.closeLogs();
    }
}
