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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;



@TeleOp(name="Lead", group="1")

public class Lead extends LinearOpMode {


    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor midDrive = null;
    private DcMotor intake = null;
    private DcMotor macro = null;
    private CRServo skanan = null;
    private CRServo skiri = null;
    private DcMotor lift = null;
    private DcMotor supp = null;
    private DcMotor dock = null;
    private TouchSensor touch = null;

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
        macro = hardwareMap.get(DcMotor.class, "macro");
        skanan = hardwareMap.get(CRServo.class, "skanan");
        skiri = hardwareMap.get(CRServo.class, "skiri");
        lift = hardwareMap.get(DcMotor.class, "lift");
        supp = hardwareMap.get(DcMotor.class, "supp");
        dock = hardwareMap.get(DcMotor.class, "dock");
        touch = hardwareMap.get(TouchSensor.class, "touch");


        // Most robots need the motor or servo on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        midDrive.setDirection(DcMotor.Direction.REVERSE);
        skanan.setDirection(CRServo.Direction.REVERSE);
        skiri.setDirection(CRServo.Direction.REVERSE);

        //Brake//

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //midDrive.setZeroPowerBehavior(DcMotor. ZeroPowerBehavior.BRAKE);
        macro.setZeroPowerBehavior(DcMotor. ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor. ZeroPowerBehavior.BRAKE);
        supp.setZeroPowerBehavior(DcMotor. ZeroPowerBehavior.BRAKE);
        dock.setZeroPowerBehavior(DcMotor. ZeroPowerBehavior.BRAKE);


        //Re-Set Servo//

        //skanan.setPosition(1);
        //skiri.setPosition(0);


        // Setup a variable

        double leftPower1;
        double rightPower1;
        double midPower1;
        double drive1;
        double turn1;
        double leftPower2;
        double rightPower2;
        double midPower2;
        double drive2;
        double turn2;
        double speed = 0;
        boolean changeDriver = false;


        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {


            //Switch Driver//

            if(gamepad1.start == true)
            {
                changeDriver = false;
            }else if(gamepad2.start == true)
            {
                changeDriver = true;
            }


            //Move Slowly//

            if(gamepad1.x == true || gamepad2.x == true)
            {
                speed = 0.5;
            }else if(gamepad1.x == false || gamepad2.x == false)
            {
                speed = 1;
            }


            //Drive//

            if(changeDriver == false)
            {
                drive1 = -gamepad1.left_stick_y;
                turn1  =  gamepad1.left_stick_x;
                leftPower1    = Range.clip(drive1 + turn1, -speed, speed) ;
                rightPower1   = Range.clip(drive1 - turn1, -speed, speed) ;
                midPower1     = gamepad1.right_stick_x;
                leftDrive.setPower(leftPower1);
                rightDrive.setPower(rightPower1);
                midDrive.setPower(midPower1);
            }else if(changeDriver == true)
            {
                drive2 =  gamepad2.left_stick_y;
                turn2  = -gamepad2.left_stick_x;
                leftPower2    = Range.clip(drive2 - turn2, -speed, speed) ;
                rightPower2   = Range.clip(drive2 + turn2, -speed, speed) ;
                midPower2     = -gamepad2.right_stick_x;
                leftDrive.setPower(leftPower2);
                rightDrive.setPower(rightPower2);
                midDrive.setPower(midPower2);
            }


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


            //Macro Intake Servo//

            if(gamepad2.left_bumper == true)
            {
                skanan.setPower(-1);
                skiri.setPower(1);
            }else if(gamepad2.left_trigger == true)
            {
                skanan.setPower(1);
                skiri.setPower(-1);
            }else
            {
                skanan.setPower(0);
                skiri.setPower(0);
            }
                
            
            //Macro Intake DcMotor//
            
            if(gamepad2.dpad_up == true)
            {
                macro.setPower(-1);
            }else if(gamepad2.dpad_down == true)
            {
                macro.setPower(1);
            }else
            {
                macro.setPower(0);
            }


            //Lifter//
            if(gamepad2.right_bumper == true && gamepad2.right_trigger == true)
            {
                lift.setPower(1);
                supp.setPower(-1);
            }else if(gamepad2.right_bumper == true)
            {
                lift.setPower(-1);
                supp.setPower(1);
            }else if(gamepad2.right_trigger == true)
            {
                lift.setPower(0.5);
                supp.setPower(-0.5);
            }else
            {
                lift.setPower(0);
                supp.setPower(0);
            }


            //Dock//
            if(gamepad1.left_bumper == true)
            {
                dock.setPower(1);
            }else if(gamepad1.left_trigger == true)
            {
                dock.setPower(-1);
            }else
            {
                dock.setPower(0);
            }


            // Show the elapsed game time.

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
//MNRF// 
