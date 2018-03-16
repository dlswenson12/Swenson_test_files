/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Team 12624 Telop", group="Linear Opmode")
//@Disabled
public class Team_12624_telop_code extends LinearOpMode {

    // Declare OpMode members.
    HardwareTeam12624         robot   = new HardwareTeam12624();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot.leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        robot.rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        robot.elevatorDrive  = hardwareMap.get(DcMotor.class, "elevatorDrive");
        robot.leftTopClaw  = hardwareMap.get(Servo.class, "leftTopClaw");
        robot.leftBottomClaw  = hardwareMap.get(Servo.class, "leftBottomClaw");
        robot.rightTopClaw  = hardwareMap.get(Servo.class, "righttopClaw");
        robot.rightBottomClaw  = hardwareMap.get(Servo.class, "rightBottomClaw");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        robot.leftDrive.setDirection(DcMotor.Direction.FORWARD);
        robot.rightDrive.setDirection(DcMotor.Direction.REVERSE);
        robot.elevatorDrive.setDirection(DcMotor.Direction.FORWARD);


//  put the following in the play section
        double          clawOffset      = 0;                       // Servo mid position
        final double    CLAW_SPEED      = 0.02 ;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double elevatorPower;
            double armPower;

        //control elements
            leftPower  = -gamepad1.left_stick_y ;
            rightPower = -gamepad1.right_stick_y ;
            elevatorPower  = -gamepad2.left_stick_y ;




            // Send calculated power to wheels
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);
            robot.elevatorDrive.setPower(elevatorPower);

            //control servos

            if (gamepad1.x) {
                robot.leftTopClaw.setPosition(robot.leftTopClaw.getPosition() + CLAW_SPEED);
                robot.leftBottomClaw.setPosition(robot.leftBottomClaw.getPosition() + CLAW_SPEED);
                robot.rightTopClaw.setPosition(robot.rightTopClaw.getPosition() - CLAW_SPEED);
                robot.rightBottomClaw.setPosition(robot.rightBottomClaw.getPosition() - CLAW_SPEED);

            }
            else if (gamepad1.y)
            {
                robot.leftTopClaw.setPosition(robot.leftTopClaw.getPosition() + CLAW_SPEED);
                robot.leftBottomClaw.setPosition(robot.leftBottomClaw.getPosition() + CLAW_SPEED);
                robot.rightTopClaw.setPosition(robot.rightTopClaw.getPosition() - CLAW_SPEED);
                robot.rightBottomClaw.setPosition(robot.rightBottomClaw.getPosition() - CLAW_SPEED);


            }



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Other Motors", "elevator (%.2f)", elevatorPower);
            telemetry.update();
        }
    }
}
