package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Config;


public class Drive extends Config {
  private DcMotor motor1 = null;
  private DcMotor motor2 = null;
  private DcMotor motor3 = null;
  private DcMotor motor4 = null;
  public int m1Speed = 0;
  public int m2Speed = 0;
  public int m3Speed = 0;
  public int m4Speed = 0;

  public Drive(DcMotor m1, DcMotor m2, DcMotor m3, DcMotor m4) {
    motor1 = m1;
    motor2 = m2;
    motor3 = m3;
    motor4 = m4;
  }

  private setPower(DcMotor m, int speed) {
    m.setPower(speed * Config.driveMultiplier);

  }

  public static void DriveForward() {
    motor2.setPower(1 * Config.driveMultiplier);
    motor4.setPower(1 * Config.driveMultiplier);
    motor1.setPower(-1 * Config.driveMultiplier);
    motor3.setPower(-1 * Config.driveMultiplier);
  }

}
