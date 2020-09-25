package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.Units;

public class TilerunnerMecanumBot extends TilerunnerGtoBot
{
    public DcMotor lfMotor = null;
    public DcMotor lrMotor = null;
    public DcMotor rfMotor = null;
    public DcMotor rrMotor = null;

    private static final String TAG = "SJH_MEC";

    public TilerunnerMecanumBot()
    {
        super();

        name= "MEC";

        COUNTS_PER_MOTOR_REV = 28;
        DRIVE_GEARS = new double[]{19.2, 1.0};

        WHEEL_DIAMETER_INCHES = 4.0;
        TUNE = 1.00;

        BOT_WIDTH  = 14.9f; //Wheel width
        BOT_LENGTH = 18.0f;

        REAR_OFFSET = 9.0f;
        FRNT_OFFSET = BOT_LENGTH - REAR_OFFSET;

        CAMERA_X_IN_BOT = 0.0f * (float)Units.MM_PER_INCH;
        CAMERA_Y_IN_BOT = 0.0f * (float)Units.MM_PER_INCH;
        CAMERA_Z_IN_BOT = 0.0f * (float)Units.MM_PER_INCH;

        LEFT_DIR  = DcMotorSimple.Direction.REVERSE;
        RIGHT_DIR = DcMotorSimple.Direction.FORWARD;

        gyroInverted = false;
    }

    @Override
    protected void initDriveMotors()
    {
        RobotLog.dd(TAG, "Initializing mecanum drive motors");
        try  //Drivetrain
        {
            lfMotor = hwMap.dcMotor.get("FL");
            lrMotor = hwMap.dcMotor.get("BL");
            rfMotor = hwMap.dcMotor.get("FR");
            rrMotor = hwMap.dcMotor.get("BR");
            motors.put("FR", rfMotor);
            motors.put("BR", rrMotor);
            motors.put("BL", lrMotor);
            motors.put("FL", lfMotor);
            leftMotors.add(numLmotors++, lfMotor);
            leftMotors.add(numLmotors++, lrMotor);
            rightMotors.add(numRmotors++, rfMotor);
            rightMotors.add(numRmotors++, rrMotor);
            capMap.put("drivetrain", true);
        }
        catch (Exception e)
        {
            RobotLog.ee("SJH", "ERROR get hardware map\n" + e.toString());
        }

        int mnum = 0;
        for(DcMotor mot : motors.values())
        {
            if(mot != null)
            {
                mot.setPower(0);
                mot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                mot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                mot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                mot.setDirection(LEFT_DIR);

                if (mot instanceof DcMotorEx)
                {
                    DcMotorEx lex = (DcMotorEx) mot;
                    PIDFCoefficients pid;
                    pid = lex.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
                    RobotLog.dd(TAG, "RUN_TO_POS Motor %d PIDs. P:%.2f I:%.2f D:%.2f",
                            mnum, pid.p, pid.i, pid.d);
                    pid = lex.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                    RobotLog.dd(TAG, "RUN_USING_ENC Motor %d PIDs. P:%.2f I:%.2f D:%.2f",
                            mnum, pid.p, pid.i, pid.d);
                    //pid = lex.getPIDCoefficients(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    //RobotLog.dd(TAG, "RUN_WITHOUT_ENC Motor %d PIDs. P:%.2f I:%.2f D:%.2f",
                    //        mnum, pid.p, pid.i, pid.d);
                }
            }
            mnum++;
        }

        rfMotor.setDirection(RIGHT_DIR);
        rrMotor.setDirection(RIGHT_DIR);
    }

    @Override
    protected void initCollectorLifter()
    {
    }

    @Override
    protected void initPushers()
    {
    }
}
