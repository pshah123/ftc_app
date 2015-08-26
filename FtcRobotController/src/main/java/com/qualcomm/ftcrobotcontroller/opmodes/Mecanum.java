/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import android.os.Handler;
import android.os.SystemClock;
import android.util.Log;
import android.view.ViewParent;

import java.sql.Time;
import java.util.concurrent.*;
import java.util.Timer;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

public class Mecanum extends OpMode {
	Handler timeHandler = new Handler();
	int frontleft = 1, frontright = 2, backleft = 3, backright = 4;
	DcMotor[] motor = new DcMotor[4];
	Servo flag, clamp, dump;
	public float x, y, z, pwr;
	int[] leftMotors = {0, 2};
	int[] rightMotors = {1, 3};
	int[][] diagonalPairs = {{0, 3}, {1, 2}};
	GyroSensor gyro;

	public void wait1Msec(int time,Runnable r){
		timeHandler.postDelayed(r,time);
	}

	float initTime, delTime=0;
	float curRate = 0;
	float currHeading = 0, prevHeading = 0;

	Runnable getHeading = new Runnable() {
		public void run() {
			while(true) {
				initTime = System.currentTimeMillis();
				curRate = (float) gyro.getRotation();
				if (Math.abs(curRate) > 3) //sets deadzones for gyroscope
				{
					prevHeading = currHeading;
					currHeading = prevHeading + curRate * delTime;
					if (currHeading > 360) currHeading -= 360;
					else if (currHeading < 0) currHeading += 360;
				}
				delTime = (System.currentTimeMillis() - initTime) / 1000;
			}
		}
	};

	public Mecanum() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void start() {
		motor[frontleft] = hardwareMap.dcMotor.get("front_left");
		motor[frontright] = hardwareMap.dcMotor.get("front_right");
		motor[backleft] = hardwareMap.dcMotor.get("back_left");
		motor[backright] = hardwareMap.dcMotor.get("back_right");
		motor[frontright].setDirection(DcMotor.Direction.REVERSE);
		motor[backright].setDirection(DcMotor.Direction.REVERSE);
		flag = hardwareMap.servo.get("flag");
		dump = hardwareMap.servo.get("dump");
		clamp = hardwareMap.servo.get("clamp");
		gyro = hardwareMap.gyroSensor.get("gyro");
		float initial = 0;
		for(int i = 0; i < 100; i++){
			initial += gyro.getRotation();
		}
		initial = initial / 100;
		FutureTask gyroUpdater = new FutureTask(getHeading, new String("Placeholder"));
		gyroUpdater.run();
		//now started getting heading
	}



	@Override
	public void loop() {

		y = gamepad1.left_stick_y;
		x = gamepad1.left_stick_x;
		z = gamepad1.right_stick_x;
		pwr = y + z;
		for (int motorNumber : diagonalPairs[1])
			motor[motorNumber].setPower(Range.clip(pwr - x, -1, 1));
		for (int motorNumber : diagonalPairs[0])
			motor[motorNumber].setPower(Range.clip(pwr + x, -1, 1));

		telemetry.addData("Text", "*** Robot Data***");
		Log.d("Gyro:","GYROKEY GYRO IS AT "+ currHeading);
	}

	@Override
	public void stop() {

	}
}