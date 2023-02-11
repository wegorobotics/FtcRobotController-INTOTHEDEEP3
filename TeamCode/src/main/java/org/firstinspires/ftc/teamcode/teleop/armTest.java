package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.arm;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.wrist;
import org.firstinspires.ftc.teamcode.teamUtil.gamepadEX.button;
import org.firstinspires.ftc.teamcode.teamUtil.robotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.robotConstants;

@TeleOp(name="Servo Arm Test", group="TEST")
public class armTest extends OpMode{
    private robotConfig r;
    private boolean armState = true;

    private double intakePosition = 0;

    Telemetry.Item intakePositionData;


    @Override
    public void init() {
        r = robotConfig.getInstance(this);
        r.initSystems(robotConstants.configuredSystems.ARM, robotConstants.configuredSystems.WRIST, robotConstants.configuredSystems.INTAKE);
    }

    @Override
    public void init_loop() {
        //vision detect loop goes here
    }

    @Override
    public void start() {
        //set claw to halfway point goes here
        r.telemetry.clear();
        intakePositionData = telemetry.addData("intake position", "");
        r.gamepadEX1.a.debounce(button.debouncingType.BOTH, 0);
        r.gamepadEX1.b.debounce(button.debouncingType.BOTH, 0);
    }

    @Override
    public void loop() {

        r.commandControl.conditionalAction(() -> r.gamepadEX1.a.onPress(), () -> armState = !armState);

        /*
        r.commandControl.conditionalAction(() -> r.gamepadEX1.dpad_up.onPress(), () -> intakePosition += 0.1);
        r.commandControl.conditionalAction(() -> r.gamepadEX1.dpad_down.onPress(), () -> intakePosition -= 0.1);
         */

        if(r.gamepadEX1.b.isPressed()){
            r.intake.presetTargetPosition(intake.intakePos.OPEN);
        }
        else{
            r.intake.presetTargetPosition(intake.intakePos.CLOSED);
        }

        if(intakePosition > 1){
            intakePosition = 1;
        } else if (intakePosition < 0) {
            intakePosition = 0;
        }

        if(armState){
            r.arm.presetTargetPosition(arm.armPos.FRONT);
            r.wrist.presetTargetPosition(wrist.wristPos.FRONT);
        }
        else{
            r.arm.presetTargetPosition(arm.armPos.BACK);
            r.wrist.presetTargetPosition(wrist.wristPos.BACK);
        }

        //r.intake.freeTargetPosition(intakePosition);

        //intakePositionData.setValue(intakePosition);

        intakePositionData.setValue(r.gamepadEX1.b.isPressed());

        r.systemsEndLoopUpdate();
    }
}
