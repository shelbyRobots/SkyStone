package org.firstinspires.ftc.teamcode.field;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.firstinspires.ftc.teamcode.util.Segment;

import java.util.Vector;

@SuppressWarnings("unused")
public class SkyRoute extends Route
{
    private static final String TAG = "SJH_RRP";

    @Override
    protected Vector<Point2d> initPoints()
    {
        RobotLog.dd(TAG, "In RoroRoute initPoints alliance=%s startPos=%s goForTwo=%s",
                alliance, startPos, goForTwo);
        Vector<Point2d> points = new Vector<>(MAX_SEGMENTS);

        //convenience declarations to make call params shorter
        ShelbyBot.DriveDir fwd = ShelbyBot.DriveDir.INTAKE;
        ShelbyBot.DriveDir rev = ShelbyBot.DriveDir.PUSHER;

        Segment.Action none    = Segment.Action.NOTHING;
        Segment.Action scan    = Segment.Action.SCAN_IMAGE;
        Segment.Action align   = Segment.Action.SET_ALIGN;
        Segment.Action drop    = Segment.Action.DROP;

        Segment.Action park    = Segment.Action.PARK;
        Segment.Action push    = Segment.Action.PUSH;
        Segment.Action grab   = Segment.Action.GRAB;
        Segment.TargetType encType = Segment.TargetType.ENCODER;
        Segment.TargetType colType = Segment.TargetType.COLOR;

        //if(startPos == StartPos.START_1) {
//            points.add(SkyField.RLS1);
//            addPoint(points, fwd, 0.50, 1.00, encType, scan,  SkyField.RIS1);
//            addPoint(points, fwd, 0.50, 1.00, encType, none,  SkyField.RTP1);
//            addPoint(points, fwd, 0.35, 1.00, encType, grab,  SkyField.RGP6);
//            addPoint(points, rev, 0.50, 1.00, encType, none,  SkyField.RTP1);
//            addPoint(points, fwd, 0.50, 1.00, encType, none,  SkyField.RTP2);
//            addPoint(points, fwd, 0.60, 1.00, encType, drop,  SkyField.RDP1);
//            addPoint(points, rev, 0.60, 1.00, encType, none,  SkyField.RTP3);
//            addPoint(points, fwd, 0.60, 1.00, encType, none,  SkyField.RTP4);
//            addPoint(points, fwd, 0.60, 1.00, encType, grab,  SkyField.RGP3);
//            addPoint(points, rev, 0.60, 1.00, encType, none,  SkyField.RTP5);
//            addPoint(points, fwd, 0.60, 1.00, encType, none,  SkyField.RTP6);
//            addPoint(points, fwd, 0.60, 1.00, encType, drop,  SkyField.RDP1);
//            addPoint(points, rev, 0.60, 1.00, encType, none,  SkyField.RTP7);
//            addPoint(points, fwd, 0.60, 1.00, encType, park,  SkyField.RPP1);

        points.add(SkyField.RLS1);
        addPoint(points, fwd, 0.50, 1.00, encType, scan,  SkyField.RIS1);
        addPoint(points, fwd, 0.50, 1.00, encType, none,  SkyField.RTP1);
        addPoint(points, fwd, 0.50, 1.00, encType, grab,  SkyField.RGPA);
        addPoint(points, fwd, 0.60, 1.00, encType, drop,  SkyField.RDPA);
        addPoint(points, rev, 0.60, 1.00, encType, grab,  SkyField.RGPB);
        addPoint(points, fwd, 0.60, 1.00, encType, none,  SkyField.RDPB);
        addPoint(points, rev, 0.60, 1.00, encType, park,  SkyField.RPP1);

//            if(goForTwo) {
//                 addPoint(points, rev, 0.70, 1.00, encType, push, SkyField.RRM2);
//            }
//            else
//            {
//                addPoint(points, fwd, 0.70, 1.00, encType, none, SkyField.RLDP);
//                addPoint(points, rev, 0.70, 1.00, encType, none, SkyField.RLDT);
//                addPoint(points, rev, 0.70, 1.00, encType, park, SkyField.RLPP);
//        }
//        }
//        else if(startPos == StartPos.START_2)
//        {
//            points.add(SkyField.RRLP);
//            addPoint(points, fwd, 0.25, 1.00, colType, align, SkyField.RRTP);
//            addPoint(points, fwd, 0.35, 1.00, encType, scan,  SkyField.RRTP);
//            addPoint(points, fwd, 0.50, 1.00, encType, push,  SkyField.RRM2);
//            addPoint(points, fwd, 0.60, 1.00, encType, drop,  SkyField.RRDP);
//            addPoint(points, rev, 0.60, 1.00, encType, none,  SkyField.RRR1);
//            addPoint(points, rev, 0.60, 1.00, encType, park,  SkyField.RRPP);
     //   }

        return points;
    }

    public SkyRoute(PositionOption startPos,
                    Field.Alliance alliance,
                    String robotName)
    {
        super(startPos, alliance);
    }

    private boolean goForTwo = false;

    public void setGoForTwo(boolean goForTwo)
    {
        this.goForTwo = goForTwo;
    }

    protected Point2d convertRtoB(Point2d rpt)
    {
        double bx =  rpt.getX();
        double by = -rpt.getY();
        String nm = "B" + rpt.getName().substring(1);

        RobotLog.dd(TAG, "convertRtoB %s to %s", rpt.getName(), nm);

        return new Point2d(nm, bx, by);
    }
}