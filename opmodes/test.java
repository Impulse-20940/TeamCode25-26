package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Cannon;
import org.firstinspires.ftc.teamcode.RobotBuild;
import org.firstinspires.ftc.teamcode.Wheelbase;
@TeleOp
public class test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotBuild r = new RobotBuild();
        Cannon cnn = new Cannon();
        Wheelbase wheel = new Wheelbase();
        r.init(hardwareMap, telemetry, gamepad1,
                gamepad2, null, cnn, null, wheel, this);
        waitForStart();//StartProgramm
        while(opModeIsActive()){
            while(gamepad1.dpad_left){
                r.delay(1);
                wheel.setMPower(0.5, -0.5, 0.5, -0.5);
            }
            wheel.setMPower(0, 0, 0, 0);
            wheel.setZPB();
        }
    }
}
