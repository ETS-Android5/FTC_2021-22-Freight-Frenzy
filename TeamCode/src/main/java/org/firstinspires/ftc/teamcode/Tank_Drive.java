package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//@TeleOp(name="Tank_Drive", group="TeleOp")
public class Tank_Drive extends LinearOpMode {

    @Override
    public void runOpMode() {

        ElapsedTime runtime = new ElapsedTime();
        DcMotor LB_Drive;
        DcMotor RB_Drive;
        DcMotor LF_Drive;
        DcMotor RF_Drive;
        double Tank_Left;
        double Tank_Right;
        double maxPower = 0.4;

        // ^ reading joystick left and right

        //Motor Mapping

        LB_Drive  = hardwareMap.get(DcMotor.class, "LB_Drive");
        RB_Drive = hardwareMap.get(DcMotor.class, "RB_Drive");
        LF_Drive  = hardwareMap.get(DcMotor.class, "LF_Drive");
        RF_Drive = hardwareMap.get(DcMotor.class, "RF_Drive");

        //setting the derection of motors

        LB_Drive.setDirection(DcMotor.Direction.REVERSE);
        RB_Drive.setDirection(DcMotor.Direction.FORWARD);
        LF_Drive.setDirection(DcMotor.Direction.FORWARD);
        RF_Drive.setDirection(DcMotor.Direction.REVERSE);

        // Display the Status of Code - At this point Robot is ready to Start
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Read Joystick Values for robot movement
            Tank_Left = gamepad1.left_stick_y;
            Tank_Right = gamepad1.right_stick_y;

            if (gamepad1.right_bumper) {
                maxPower = 0.7;
            }
            else{
                maxPower = 0.4;
            }

            Tank_Left = Range.clip(Tank_Left, -1.0 * maxPower, maxPower);
            Tank_Right = Range.clip(Tank_Right, -1.0 * maxPower, maxPower);

            // Setting Power to motors
            LB_Drive.setPower(Tank_Right);
            RB_Drive.setPower(Tank_Left);
            LF_Drive.setPower(Tank_Left);
            RF_Drive.setPower(Tank_Right);

            // Show the elapsed game time, wheel and intake system power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", Tank_Left, Tank_Right);
            telemetry.update();

        }

        // Set Power to Zero on all actuators
        LB_Drive.setPower(0);
        RB_Drive.setPower(0);
        LF_Drive.setPower(0);
        RF_Drive.setPower(0);

    }
}
