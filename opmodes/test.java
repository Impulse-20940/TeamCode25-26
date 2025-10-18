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
        r.init(hardwareMap, telemetry, gamepad1,
                gamepad2, null, cnn, null, null, this);
        waitForStart();//StartProgramm
        while(opModeIsActive()){
            double stick = gamepad1.left_stick_y;
            cnn.cannon_control(stick);
            telemetry.addData("Stick is", "%4f", stick);
            telemetry.addData("Motor is", "%4f %4f", cnn.c1.getPower(),
                                                                    cnn.c2.getPower());
            telemetry.update();
        }
    }
}
