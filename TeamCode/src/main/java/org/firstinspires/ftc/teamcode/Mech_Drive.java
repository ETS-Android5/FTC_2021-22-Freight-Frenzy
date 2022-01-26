package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Mech_Drive", group="TeleOp")
public class Mech_Drive extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Making varibals

        ElapsedTime runtime = new ElapsedTime();
        DcMotor LB_Drive;
        DcMotor RB_Drive;
        DcMotor LF_Drive;
        DcMotor RF_Drive;
        DcMotor Intake;
        DcMotor Spinner;
        DcMotor elevator;
        Servo elevator_tilt;
        double drive = 0;
        double turn=  0;
        double mech = 0;
        double RF_Mech = 0;
        double LF_Mech = 0;
        double RB_Mech = 0;
        double LB_Mech = 0;
        double maxPower = 0.25;
        double dead_zone = 0.45;
        double Intake_Toggle = 0;
        double Intake_wait = 0;
        double Spinner_Power = 0;
        double elevator_power = 0;
        double elv_tilt_wait = 0;
        double elv_tilt_toggle = 1;



        ElapsedTime Spinner_Timer = new ElapsedTime(4000);
        // ^ reading joystick left and right

        //Motor Mapping

        LB_Drive  = hardwareMap.get(DcMotor.class, "LB_Drive");
        RB_Drive = hardwareMap.get(DcMotor.class, "RB_Drive");
        LF_Drive  = hardwareMap.get(DcMotor.class, "LF_Drive");
        RF_Drive = hardwareMap.get(DcMotor.class, "RF_Drive");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Spinner = hardwareMap.get(DcMotor.class, "Spinner");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        elevator_tilt = hardwareMap.get(Servo.class, "elevator_tilt");

        //setting the direction of motors

        LB_Drive.setDirection(DcMotor.Direction.REVERSE);
        RB_Drive.setDirection(DcMotor.Direction.FORWARD);
        LF_Drive.setDirection(DcMotor.Direction.REVERSE);
        RF_Drive.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.REVERSE);
        Spinner.setDirection(DcMotor.Direction.FORWARD);
        elevator_tilt.setPosition(0.3);

        // Display the Status of Code - At this point Robot is ready to Start
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Read Joystick Values for robot movement
            drive = gamepad1.left_stick_y;
            mech = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;
        //speed boost
            if (gamepad1.right_trigger > 0.5) {
                maxPower = 1;
            }
            else if (gamepad1.right_bumper) {
                maxPower = 0.6;
            }
            else if(gamepad1.left_bumper){
                maxPower = 0.1;
            }
            else{
                maxPower = 0.25;
            }

            // Deadzone code

            if ((drive <= dead_zone && drive > 0.0) || (drive >= -dead_zone && drive < 0.0)){
                drive = 0;
            }
            if ((mech <= dead_zone && mech > 0.0) || (mech >= -dead_zone && mech < 0.0)){
                mech = 0;
            }
            if ((turn <= dead_zone && turn > 0.0) || (turn >= -dead_zone && turn < 0.0)){
                turn = 0;
            }
            //Toggle Code For Intake

            if (gamepad2.a && Intake_wait == 0) {
                Intake_wait = 1;
                if (Intake_Toggle == 0) {
                 Intake_Toggle = -1.0;
             }
             else {
                 Intake_Toggle = 0;
             }
            }

            if (gamepad2.a == false){
                Intake_wait = 0;
            }

            //spinner code

           // if (gamepad2.y) {
           //     Spinner_Power = 0.1;
          //      Spinner_Timer.reset();
           // }

           // if(Spinner_Timer.milliseconds() > 2000){
          //      Spinner_Power = 0.4;
          //  }

           // if(Spinner_Timer.milliseconds() > 3500){
           //     Spinner_Power = 0;
            //}

            if (gamepad2.x) {
                Spinner_Power = -0.3;
                Spinner_Timer.reset();
            }
            if (gamepad2.y) {
                Spinner_Power = 0;
                Spinner_Timer.reset();
            }
            //tilt elevator servo

            if (gamepad2.left_bumper && elv_tilt_wait == 0) {
                elv_tilt_wait = 1;

                if (elv_tilt_toggle == 0) {
                    elv_tilt_toggle = 1;
                    elevator_tilt.setPosition(0.1);
                }
                else if (elv_tilt_toggle == 1) {
                    elv_tilt_toggle = 2;
                    elevator_tilt.setPosition(0.3);
                }
                else {
                    elv_tilt_toggle = 0;
                    elevator_tilt.setPosition(0.7);
                }
            }

            if (gamepad2.left_bumper == false){
                elv_tilt_wait = 0;
            }

            // Elevator up and down

            elevator_power = gamepad2.left_stick_y;

            // Elevator dead zone
            if ((elevator_power <= dead_zone && elevator_power > 0.0) || (elevator_power >= -dead_zone && elevator_power < 0.0)) {
                elevator_power = 0;
            }
            elevator.setPower(elevator_power);
            // Driving commands

            LF_Mech = Range.clip(drive - mech - turn, -1.0 * maxPower, maxPower);
            RF_Mech = Range.clip(drive + mech + turn, -1.0 * maxPower, maxPower);
            RB_Mech = Range.clip(drive - mech + turn, -1.0 * maxPower, maxPower);
            LB_Mech = Range.clip(drive + mech - turn, -1.0 * maxPower, maxPower);

            // Setting Power to motors
            LB_Drive.setPower(LB_Mech);
            RB_Drive.setPower(RB_Mech);
            LF_Drive.setPower(LF_Mech);
            RF_Drive.setPower(RF_Mech);
            Intake.setPower(Intake_Toggle);
            Spinner.setPower(Spinner_Power);

            // Show the elapsed game time, wheel and intake system power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front Motors", "left (%.2f), right (%.2f)", LF_Mech, RF_Mech);
            telemetry.addData("Back Motors", "left (%.2f), right (%.2f)", LB_Mech, RB_Mech);
            telemetry.addData("Toggle_Elevator_tilt", "(%.2f)", elv_tilt_toggle);
            telemetry.update();
        }

        // Set Power to Zero on all actuators
        LB_Drive.setPower(0);
        RB_Drive.setPower(0);
        LF_Drive.setPower(0);
        RF_Drive.setPower(0);
        Intake.setPower(0);
        Spinner.setPower(0);
        elevator_tilt.setPosition(0.1);
        elevator.setPower(0);
    }
}
