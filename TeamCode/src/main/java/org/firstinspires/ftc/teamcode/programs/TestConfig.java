package org.firstinspires.ftc.teamcode.programs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.base.Components;
import java.util.ArrayList;
import java.util.Arrays;

public class TestConfig implements Components.RobotConfig {
    @Override
    public ArrayList<Components.Actuator<?>> getActuators() {
        return new ArrayList<>(Arrays.asList(
                leftFront, leftRear, rightFront, rightRear
        ));
    }
    public static Components.BotMotor leftFront = new Components.BotMotor("leftFront", DcMotorSimple.Direction.REVERSE);
    public static Components.BotMotor leftRear = new Components.BotMotor("leftRear", DcMotorSimple.Direction.REVERSE);
    public static Components.BotMotor rightFront = new Components.BotMotor("rightFront", DcMotorSimple.Direction.FORWARD);
    public static Components.BotMotor rightRear = new Components.BotMotor("rightRear", DcMotorSimple.Direction.FORWARD);
    
}