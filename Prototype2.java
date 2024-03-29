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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Prototype2", group="1")

public class Prototype2 extends LinearOpMode {


    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor midDrive = null;
    private DcMotor intake = null;
    private DcMotor shooter = null;
    private DcMotor fold = null;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        midDrive = hardwareMap.get(DcMotor.class, "mid_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        fold = hardwareMap.get(DcMotor.class, "fold");


        // Most robots need the motor or servo on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        midDrive.setDirection(DcMotor.Direction.REVERSE);


        //Brake//

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fold.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //midDrive.setZeroPowerBehavior(DcMotor. ZeroPowerBehavior.BRAKE);


        // Setup a variable

        double leftPower;
        double rightPower;
        double midPower;
        double drive;
        double turn;


        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {

            //Drive//
            
                drive = -gamepad1.left_stick_y;
                turn  =  gamepad1.left_stick_x;
                leftPower    = Range.clip(drive + turn, -1, 1) ;
                rightPower   = Range.clip(drive - turn, -1, 1) ;
                midPower     = gamepad1.right_stick_x;
                leftDrive.setPower(leftPower);
                rightDrive.setPower(rightPower);
                midDrive.setPower(midPower);
                
            //Micro Intake//

            if(gamepad1.right_bumper == true)
            {
                intake.setPower(1);
            }else if(gamepad1.right_trigger == true)
            {
                intake.setPower(-1);
            }else
            {
                intake.setPower(0);
            }

            //Fold in & Fold out//

            if(gamepad1.left_bumper == true)
            {
                intake.setPower(1);
            }else if(gamepad1.left_trigger == true)
            {
                intake.setPower(-1);
            }else
            {
                intake.setPower(0);
            }

            // Show the elapsed game time.

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
//MNRF// 
