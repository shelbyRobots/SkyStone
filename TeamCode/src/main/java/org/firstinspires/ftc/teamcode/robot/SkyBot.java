package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.Units;

@SuppressWarnings({"FieldCanBeLocal", "unused", "WeakerAccess"})
public class SkyBot extends TilerunnerGtoBot {
    private final String TAG = "SJH SkyRobot";

    private DcMotor _liftyBoi = null;
    //5202 series yellow jacket motor w/ 5.2:1 gear box has 145.6 cpr of output shaft
    //  1150 rpm (19.17 rps) no load speed
    //
    // Replaced with 13.7:1 gear box motor with 383.6 cpr
    //
    //lift is made with 1:1 gear between motor output shaft and lead screw
    //
    //gobilda lead screw has 7.9in range, 2mm pitch, and 4 start
    //lead=2mm pitch * 4 start = 8 mm/lead screw rev
    //19.17rps * 8mm/rev = 153mm/s no load lift speed (6in/s)
    //201mm (7.9in) range in 201/153=1.31 s
    //201mm / 8mm/rev = 25.125rev
    //25.125rev * 145.6 cpr = 3658 counts

    private final double LIFTER_CPER = 28; //quad encoder cnts/encoder rev
    private final double LIFTER_INT_GEAR = 13.7;  // 5.2;
    private final double LIFTER_CPOR = LIFTER_CPER * LIFTER_INT_GEAR; //383.6 //145.6 cnts/outShaftRev
    private final double LIFTER_EXT_GEAR = 1.0;
    private final double LIFTER_PITCH = 2.0; //2.0mm/pitch
    private final int    LIFTER_STARTS = 4; //pitch/rev
    private final double LIFTER_LEAD = LIFTER_PITCH * LIFTER_STARTS; //8 mm/rev
    private final double LIFTER_CPMM = LIFTER_EXT_GEAR * LIFTER_CPOR / LIFTER_LEAD;
    private final double LIFTER_CPI = LIFTER_CPMM * 25.4;
    @SuppressWarnings("FieldCanBeLocal")
    private final int    LIFTER_THRESH = (int) (0.1 * LIFTER_CPI);
    private final double LIFTER_STOW =  0.0;
    private final double LIFTER_GRAB = -2.0;
    private final double LIFTER_MOVE = -1.5;
    private final double LIFTER_REL1 =  0.0;
    private final double LIFTER_REL2 =  5.0;
    private final int STOW_CNTS = (int) (LIFTER_CPI * LIFTER_STOW);
    private final int GRAB_CNTS = (int) (LIFTER_CPI * LIFTER_STOW);
    private final int MOVE_CNTS = (int) (LIFTER_CPI * LIFTER_MOVE);
    private final int REL1_CNTS = (int) (LIFTER_CPI * LIFTER_REL1);
    private final int REL2_CNTS = (int) (LIFTER_CPI * LIFTER_REL2);

    private Servo    lplatch    = null;
    private Servo    rplatch    = null;
    @SuppressWarnings("FieldCanBeLocal")
    private final double LPLATCH_STOW = 0.98;
    @SuppressWarnings("FieldCanBeLocal")
    private final double LPLATCH_PRE  = 0.5;
    @SuppressWarnings("FieldCanBeLocal")
    private final double LPLATCH_GRAB = 0.38;
    @SuppressWarnings("FieldCanBeLocal")
    private final double RPLATCH_STOW = 0.98;
    @SuppressWarnings("FieldCanBeLocal")
    private final double RPLATCH_PRE  = 0.5;
    @SuppressWarnings("FieldCanBeLocal")
    private final double RPLATCH_GRAB = 0.38;

    @SuppressWarnings("FieldCanBeLocal")
    private final double GRIPPER_STOW = 0.98;
    @SuppressWarnings("FieldCanBeLocal")
    private final double GRIPPER_OPEN = 0.5;
    @SuppressWarnings("FieldCanBeLocal")
    private final double GRIPPER_PRE  = 0.38;
    @SuppressWarnings("FieldCanBeLocal")
    private final double GRIPPER_GRAB = 0.38;



    private final int    EXTND_COUNTS_PER_MOTOR_REV = 4;
    private final double EXTND_GEAR_ONE   = 72;
    private final double EXTND_WHEEL_DIAM = 2.0625; //1.5; - spool diam
    private final double EXTND_CPR        = ELEV_COUNTS_PER_MOTOR_REV * ELEV_GEAR_ONE;
    private final double EXTND_SCALE      = 1.0;
    public double EXTND_CPI = ELEV_CPR/(Math.PI * ELEV_WHEEL_DIAM)/LIFT_SCALE;
    private int MIN_EXTND_CNT = 0;

    private DigitalChannel armIndexSensor = null;

    public  DcMotor  armRotate   = null;
    public  DcMotor  armExtend   = null;

    public SkyBot() {
        super();

        CAMERA_X_IN_BOT = 4.0f * (float) Units.MM_PER_INCH;
        CAMERA_Y_IN_BOT = 0.0f * (float) Units.MM_PER_INCH;
        CAMERA_Z_IN_BOT = 3.0f * (float) Units.MM_PER_INCH;
    }

    public SkyBot(String name)
    {
        this();
        this.name = name;
    }

    @Override
    public void init(LinearOpMode op, boolean initDirSensor) {
        computeCPI();

        initOp(op);
        initDriveMotors();
        initCollectorLifter();
        initSensors(initDirSensor);
        initArm();
        initHolder();
        initCapabilities();
    }

    @Override
    public void initCollectorLifter() //elev and stone gripper
    {
        initGripper();
        initLifter();
    }

    @Override
    public void initArm() //rotate, extend
    {
        RobotLog.dd(TAG, "GTO initArm");
        try
        {
            armExtend = hwMap.dcMotor.get("armExtend");
            armRotate  = hwMap.dcMotor.get("armPitch");
            armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            capMap.put("arm", true);
        }
        catch(Exception e)
        {
            RobotLog.ee(TAG, "ERROR in initArm\n" + e.toString());
        }
    }

    @Override
    public void initHolder() //Platform grabber
    {
        boolean success = true;
        try
        {
            rplatch = hwMap.servo.get("rplatch");
        }
        catch (Exception e)
        {
            return;
        }

        try
        {
            lplatch = hwMap.servo.get("lplatch");
        }
        catch (Exception e)
        {
            return;
        }

        capMap.put("holder",  true);
    }

    @Override
    public void initSensors() {
        super.initSensors();
    }

    @SuppressWarnings("UnusedReturnValue")
    private boolean initGripper()
    {
        try {
            gripper = hwMap.servo.get("gripper");

            //if extended range is wanted (and available) on servo,
            //keep block below
            //if standard range is wanted, comment block below
            if (gripper.getController() instanceof ServoControllerEx)
            {
                // Set the rotation servo for extended PWM range
                ServoControllerEx srvCntrlrEx =
                        (ServoControllerEx) gripper.getController();
                int gPort = gripper.getPortNumber();
                PwmControl.PwmRange range =
                        new PwmControl.PwmRange(500, 2500);
                srvCntrlrEx.setServoPwmRange(gPort, range);
            }
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR in initGripper\n" + e.toString());
            return false;
        }

        capMap.put("collector",  true);
        return true;
    }

    @SuppressWarnings("UnusedReturnValue")
    private boolean initLifter()
    {
        try {
            _liftyBoi = hwMap.dcMotor.get("liftyboi");

            _liftyBoi.setDirection(DcMotorSimple.Direction.REVERSE);
            //_liftyBoi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            _liftyBoi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            _liftyBoi.setPower(0.0f);
            _liftyBoi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR in initLifter\n" + e.toString());
            return false;
        }

        capMap.put("lifter", true);
        return true;
    }

    public boolean isElevTouchPressed()
    {
        return false;
    }
}
