package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "Motor-Servo Tester", group = "Tests")
public class servoTester extends OpMode {

    public static String motorHardwareName = "";
    public static String servoHardwareName = "";

    public static double outputServo = 0;
    public static double outputMotor = 0;

    Servo servo;
    DcMotor motor;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, servoHardwareName);
        motor = hardwareMap.get(DcMotor.class, motorHardwareName);
    }

    @Override
    public void loop() {
        servo.setPosition(outputServo);
        motor.setPower(outputMotor);
    }
}
