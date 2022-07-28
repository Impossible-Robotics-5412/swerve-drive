package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbase.command.SharedCommand;
import org.firstinspires.ftc.teamcode.common.freightfrenzy.Alliance;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

@TeleOp
public class TestTeleOP extends CommandOpMode {
    private Robot robot;
    private boolean intake = true;
    private BooleanSupplier outtake;
    private Consumer<Boolean> done;
    private DoubleSupplier linkage, arm;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);

        outtake = () -> gamepad1.right_bumper;
        done = (a) -> intake = a;
        linkage = () -> gamepad1.right_trigger;
        arm = () -> gamepad1.left_trigger;

        robot.turret.middle();
        robot.arm.linkageIn();
        robot.bucket.in();
    }

    @Override
    public void run() {
        super.run();
        robot.arm.loop();

        if (gamepad1.a) {
            schedule(new SharedCommand(robot, Alliance.RED, outtake, linkage, arm, done));
        }
    }
}
