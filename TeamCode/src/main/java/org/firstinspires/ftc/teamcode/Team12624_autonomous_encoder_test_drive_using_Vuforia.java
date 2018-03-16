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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - close gripper on block
 *   - lift up block with elevator
 *   - Drive Forwards 24in
 *   - Open claw to drop block
 *   -Drive Backwards 3in
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Team12624_encoder_test_vuforia", group="Team 12624")
//@Disabled
public class Team12624_autonomous_encoder_test_drive_using_Vuforia extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareTeam12624         robot   = new HardwareTeam12624();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double     LIFT_SPEED              = 0.25;
    double          CLAW_MIDPSN      = 0.5;// Servo mid position
    final double    CLAW_SPEED      = 0.02 ;


    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    //@Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //* To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
        //* If no camera monitor is desired, use the parameterless constructor instead (commented out below).

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        //Vuforia license
        parameters.vuforiaLicenseKey = "AYiKw9r/////AAAAGYbhLmxEt0zEv7+SGIQhKedxhxk6E/HDzDaKJ30TKlWq0sHSNbXGfvg4aOce9jmC/E0vIkuyQfdtOGEag65Dt25C72Dc6HbAmOnC32SQj5+mr5P0IUyqu2++iCCedH3F9lRLy0T+011r9m/54Bw56EKf6sDHCJcN3Sx2KlCJmgxY33OBL7Z0pK5+SwU5i05mBVmudxDEndnogNbV8192FmfaxgrZeADpITsM1EEGpvh044Ptt+sH8Uy0d8I6jgY1pAdDPbste8mF2BuYuMdIuMzR+Gt9iKNkfkEPyKg1KNSD7Mn+E2KWjzGfncz7lX+b34MkbinrefpxfffqXGEytUQaAnGZBL44tSF9e36mshme";

         /* We also indicate which camera on the RC that we wish to use.
                * Here we chose the back (HiRes) camera (for greater range), but
                * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        /*start tracking
          */
        relicTrackables.activate();

        while (opModeIsActive()) {

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            // close gripper on block.  Use 0 for center position
            clawmovement(0);

            //lift elevator so block is off the ground
            elevatorLift(LIFT_SPEED, 5);

            if (vuMark != RelicRecoveryVuMark.CENTER) {

                // Step through each leg of the movement
                // Note: Reverse movement is obtained by setting a negative distance (not speed)
                encoderDrive(DRIVE_SPEED, 24, 24, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout

                //Open claws to drop block.  Use .5 to Open Claw
                clawmovement(.5);

                //Back up
                encoderDrive(TURN_SPEED, -3, -3, 5.0);  // S2: Turn Right 12 Inches with 5 Sec timeout


                sleep(1000);     // pause for servos to move

                telemetry.addData("Path", "Complete");
                telemetry.update();
                }

            if (vuMark != RelicRecoveryVuMark.LEFT) {

                // Step through each leg of the movement
                // Note: Reverse movement is obtained by setting a negative distance (not speed)
                encoderDrive(DRIVE_SPEED, 20, 20, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
                encoderDrive(DRIVE_SPEED, -4, 4, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
                encoderDrive(DRIVE_SPEED, 4, 4, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
                encoderDrive(DRIVE_SPEED, 4, -4, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout


                //Open claws to drop block.  Use .5 to Open Claw
                clawmovement(.5);

                //Back up
                encoderDrive(TURN_SPEED, -3, -3, 5.0);  // S2: Turn Right 12 Inches with 5 Sec timeout


                sleep(1000);     // pause for servos to move

                telemetry.addData("Path", "Complete");
                telemetry.update();
            }

            if (vuMark != RelicRecoveryVuMark.RIGHT) {

                // Step through each leg of the movement
                // Note: Reverse movement is obtained by setting a negative distance (not speed)
                encoderDrive(DRIVE_SPEED, 20, 20, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
                encoderDrive(DRIVE_SPEED, 4, -4, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
                encoderDrive(DRIVE_SPEED, 4, 4, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
                encoderDrive(DRIVE_SPEED, -4, 4, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout

                //Open claws to drop block.  Use .5 to Open Claw
                clawmovement(.5);

                //Back up
                encoderDrive(TURN_SPEED, -3, -3, 5.0);  // S2: Turn Right 12 Inches with 5 Sec timeout


                sleep(1000);     // pause for servos to move

                telemetry.addData("Path", "Complete");
                telemetry.update();
            }

            }
        }

    public void clawmovement(double OPENorCLOSED) {

        if (opModeIsActive()) {
            robot.leftTopClaw.setPosition(CLAW_MIDPSN + OPENorCLOSED);
            robot.leftBottomClaw.setPosition(CLAW_MIDPSN + OPENorCLOSED);
            robot.rightTopClaw.setPosition(CLAW_MIDPSN - OPENorCLOSED);
            robot.rightBottomClaw.setPosition(CLAW_MIDPSN - OPENorCLOSED);
        }
    }

    public void elevatorLift(double elevatorPower, double timeoutS)  {

        if (opModeIsActive()) {
            robot.elevatorDrive.setPower(Math.abs(elevatorPower));
        }


    }



    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.leftDrive.getCurrentPosition(),
                                            robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
