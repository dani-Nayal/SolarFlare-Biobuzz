package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.programs.Beecode.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.Commands;
import org.firstinspires.ftc.teamcode.base.Components;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    Beecode robot = new Beecode();

    public  void runOpMode() throws InterruptedException {

        Components.initialize(this, robot, false, true);
        Components.activateActuatorControl();

        executor.setCommands(
            new Commands.RunResettingLoop(
                new Commands.RobotCentricMecanumCommand(
                    new Components.BotMotor[] {
                        leftFront,
                        leftBack,
                        rightFront,
                        rightBack
                    },
                        () -> (double) gamepad1.left_stick_x,
                        () -> (double) gamepad1.left_stick_y,
                        () -> (double) -gamepad1.right_stick_x
                ),
                new Commands.PressCommand(
                    new Commands.IfThen(() -> gamepad1.left_bumper,
                        new Commands.InstantCommand(() -> {
                            transferGate.setTarget(180);
                            transfer.setPower(1.0);
                        })
                    )
                ),
                new Commands.PressCommand(
                    new Commands.IfThen(() -> gamepad1.right_bumper,
                        new Commands.InstantCommand(() -> {
                            hood.setTarget(180);;
                        })
                    )
                ),
                new Commands.PressCommand(
                    new Commands.IfThen(() -> gamepad1.cross,
                        new Commands.InstantCommand(() -> {
                            ramp.setTarget(180);;
                        })
                    )
                ),
                new Commands.InstantCommand(() -> {
                    Beecode.isIntaking = gamepad1.left_trigger_pressed;
                    Beecode.isShooting = gamepad1.right_trigger_pressed;
                    Hive.updateIntake();
                    Hive.updateShooting();
                })
            )
        );
        waitForStart();
        executor.run(this::opModeIsActive);
    }
}