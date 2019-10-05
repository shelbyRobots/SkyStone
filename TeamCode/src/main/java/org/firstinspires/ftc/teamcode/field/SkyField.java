package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.image.MineralDetector;
import org.firstinspires.ftc.teamcode.util.Point2d;

@SuppressWarnings({"unused", "WeakerAccess"})
public class SkyField extends Field
{
    public SkyField()
    {
        super("Skystone", TRACKABLE_NAMES, LOCATIONS_ON_FIELD);
    }

    //Point naming key:
    //1st char: R=Red, B=Blue
    //2nd char: L=Left start, R=Right start (viewed from red side - along field X)
    //3rd-4th chars: Pt description

    private static final String TAG = " SJH_RFD";

    private static final int BLUE  = 0;
    private static final int RED   = 1;
    private static final int STRT1 = 0;
    private static final int STRT2 = 1;
    private static final int LEFT  = 0;
    private static final int CNTR  = 1;
    private static final int RGHT  = 2;

    //Red Route > Left
    //Red Left Start Point 1
    static final Point2d RLS1 = new Point2d("RLS1", -36.0,  -61.0);

    //Red Scan Pt (Image Scan Pt)
    static final Point2d RIS1 = new Point2d("RIS1", -36.0,  -50.0);
    //includes offset for veh center. actual

    //Red Stone Points (Stone center location on field)
    static final Point2d RSP1 = new Point2d("RSP1", -68.0,  -21.0); //R
    static final Point2d RSP2 = new Point2d("RSP2", -60.0,  -21.0); //R
    static final Point2d RSP3 = new Point2d("RSP3", -52.0,  -21.0); //R
    static final Point2d RSP4 = new Point2d("RSP4", -44.0,  -21.0); //R
    static final Point2d RSP5 = new Point2d("RSP5", -36.0,  -21.0); //R
    static final Point2d RSP6 = new Point2d("RSP6", -28.0,  -21.0); //R

    //Red Grab Points - will need tuning after grabber installed
    static final Point2d RGP1 = new Point2d("RGP1", -63.0,  -33.0); //R
    static final Point2d RGP2 = new Point2d("RGP2", -60.0,  -33.0); //R
    static final Point2d RGP3 = new Point2d("RGP3", -57.0,  -33.0); //R
    static final Point2d RGP4 = new Point2d("RGP4", -39.0,  -33.0); //R
    static final Point2d RGP5 = new Point2d("RGP5", -36.0,  -33.0); //R
    static final Point2d RGP6 = new Point2d("RGP6", -33.0,  -33.0); //R

    static final Point2d RGPA = new Point2d("RGPA", -36.1,  -36.0); //R
    static final Point2d RGPB = new Point2d("RGPB", -60.0,  -36.0); //R
    static final Point2d RDPA = new Point2d("RDPA",  36.0,  -36.0); //R
    static final Point2d RDPB = new Point2d("RDPB",  36.0,  -36.0); //R

    static final Point2d RTP1 = new Point2d("RTP1", -36.0,  -36.0); //R
    static final Point2d RTP2 = new Point2d("RTP2",  36.0,  -36.0); //R
    static final Point2d RTP3 = new Point2d("RTP3", RTP2);
    static final Point2d RTP4 = new Point2d("RTP4", -60.0,  -36.0); //R
    static final Point2d RTP5 = new Point2d("RTP5", RTP4);
    static final Point2d RTP6 = new Point2d("RTP6", RTP2);
    static final Point2d RTP7 = new Point2d("RTP7", RTP2);


    static final Point2d RDP1 = new Point2d("RDP1",  36.0,  -33.0); //R
    static final Point2d RDP2 = new Point2d("RDP1", RDP1); //R

    static final Point2d RPP1 = new Point2d("RPP1",   0.0,  -36.0); //R



    private static final int ALNC_RED = 0;
    private static final int ALNC_BLU = 1;
    private static final int STRT_ONE = 0;
    private static final int STRT_TWO = 1;
    private static final int STN_LEFT = 0;
    private static final int STN_CNTR = 1;
    private static final int STN_RGHT = 2;

    //private static Point2d[][][] minOffsetPts =
    //        {{{RLM3, RLM2, RLM1}, {RRM3, RRM2, RRM1}}, {{BLM3, BLM2, BLM1}, {BRM3, BRM2, BRM1}}};

    //private static Point2d[][][] minActualPts =
    //        {{{RLML, RLMC, RLMR}, {RRML, RRMC, RRMR}}, {{BLML, BLMC, BLMR}, {BRML, BRMC, BRMR}}};

//    public static Point2d getMinOffsetPt(Alliance alnc,
//                                         PositionOption startPos,
//                                         MineralDetector.Position minPos)
//    {
//        int allnc = alnc == Alliance.RED ? 0 : 1;
//        int start = startPos == Route.StartPos.START_1 ? 0 : 1;
//        int minps = 1; //CENTER or NONE
//        if      (minPos == MineralDetector.Position.LEFT) minps = 0;
//        else if (minPos == MineralDetector.Position.RIGHT) minps = 2;
//        return minOffsetPts[allnc][start][minps];
//    }
//
//    public static Point2d getMinActualPt(Alliance alnc,
//                                         PositionOption startPos,
//                                         MineralDetector.Position minPos)
//    {
//        int allnc = alnc == Alliance.RED ? 0 : 1;
//        int start = startPos == Route.StartPos.START_1 ? 0 : 1;
//        int minps = 1; //CENTER or NONE
//        if      (minPos == MineralDetector.Position.LEFT) minps = 0;
//        else if (minPos == MineralDetector.Position.RIGHT) minps = 2;
//        return minActualPts[allnc][start][minps];
//    }
//
//    public static Point2d getMineralPt(Alliance alnc,
//                                       PositionOption startPos,
//                                       MineralDetector.Position minPos)
//    {
//        Route.StartPos spos = Route.StartPos.START_1;
//        if(startPos instanceof Route.StartPos) spos = (Route.StartPos) startPos;
//        Point2d minPt = null;
//        switch(alnc)
//        {
//            case RED: {
//                switch (spos) {
//                    case START_1: {
//                        switch (minPos) {
//                            case LEFT:    minPt = RLM3; break;
//                            case RIGHT:   minPt = RLM1; break;
//                            case CENTER:  minPt = RLM2; break;
//                            case NONE:    minPt = RLM2; break;
//                        }
//                        break;
//                    }
//                    case START_2: {
//                        switch (minPos) {
//                            case LEFT:    minPt = RRM3; break;
//                            case RIGHT:   minPt = RRM1; break;
//                            case CENTER:  minPt = RRM2; break;
//                            case NONE:    minPt = RRM2; break;
//                        }
//                        break;
//                    }
//                }
//                break;
//            }
//            case BLUE: {
//                switch (spos) {
//                    case START_1: {
//                        switch (minPos) {
//                            case LEFT:    minPt = BLM3; break;
//                            case RIGHT:   minPt = BLM1; break;
//                            case CENTER:  minPt = BLM2; break;
//                            case NONE:    minPt = BLM2; break;
//                        }
//                        break;
//                    }
//                    case START_2: {
//                        switch (minPos) {
//                            case LEFT:    minPt = BRM3; break;
//                            case RIGHT:   minPt = BRM1; break;
//                            case CENTER:  minPt = BRM2; break;
//                            case NONE:    minPt = BRM2; break;
//                        }
//                        break;
//                    }
//                }
//                break;
//            }
//        }
//        return minPt;
//    }

    private static final String ASSET_NAME = "RoverRuckus";

    public static final String[] TRACKABLE_NAMES =
            {
                    "BluePerimeter_Rover",
                    "RedPerimeter_Footprint",
                    "FrontPerimeter_Craters",
                    "BackPerimeter_Space"
            };

    private static final float[][] TRACKABLE_POS = {
            scaleArr(new float[]{0.0f, N_WALL_Y, IMAGE_Z}, scale),
            scaleArr(new float[]{0.0f, S_WALL_Y, IMAGE_Z}, scale),
            scaleArr(new float[]{E_WALL_X, 0.0f, IMAGE_Z}, scale),
            scaleArr(new float[]{W_WALL_X, 0.0f, IMAGE_Z}, scale)
    };

    private static final OpenGLMatrix[] LOCATIONS_ON_FIELD =
            {
                    genMatrix(TRACKABLE_POS[0], new float[]{90.0f,   0.0f,   0.0f}),
                    genMatrix(TRACKABLE_POS[1], new float[]{90.0f,   0.0f, 180.0f}),
                    genMatrix(TRACKABLE_POS[2], new float[]{90.0f,   0.0f,  90.0f}),
                    genMatrix(TRACKABLE_POS[3], new float[]{90.0f,   0.0f, 270.0f})
            };
}
