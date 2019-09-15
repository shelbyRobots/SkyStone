/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package trclib;

public class TrcPath
{
    public static TrcPath loadPathFromCsv(String path, boolean degrees, boolean loadFromResources)
    {
        return new TrcPath(degrees, TrcWaypoint.loadPointsFromCsv(path, loadFromResources));
    }

    private TrcWaypoint[] waypoints;
    private boolean degrees;

    /**
     * Create a new TrcPath object. Must supply at least 2 entries.
     *
     * @param degrees If true, the heading values will be treated as degrees. Otherwise, they are assumed to be radians.
     * @param waypoints The array of points that will constitute this path. Cannot be null, and must have at least 2 waypoint.
     */
    public TrcPath(boolean degrees, TrcWaypoint... waypoints)
    {
        if (waypoints == null || waypoints.length <= 1)
        {
            throw new IllegalArgumentException("Waypoints cannot be null or have less than 2 entries!");
        }
        this.degrees = degrees;
        this.waypoints = waypoints;
    }

    /**
     * Get the waypoint at index i.
     *
     * @param index The index to get.
     * @return The waypoint at index i.
     */
    public TrcWaypoint getWaypoint(int index)
    {
        return waypoints[index];
    }

    /**
     * Get the number of waypoints in this path.
     *
     * @return The number of waypoints in this path.
     */
    public int getSize()
    {
        return waypoints.length;
    }

    /**
     * Return the underlying array of waypoints.
     *
     * @return The waypoint array. This is the same instance, so modifying it will affect other users of this path.
     */
    public TrcWaypoint[] getAllWaypoints()
    {
        return waypoints;
    }

    /**
     * Create a new path identical to this one, except the heading values are in degrees. If this path's headings are
     * already in degrees, just return deep copy.
     *
     * @return A TrcPath object which the same waypoints except all heading values are now in degrees, if they weren't before.
     */
    public TrcPath toDegrees()
    {
        TrcWaypoint[] waypoints = new TrcWaypoint[this.waypoints.length];
        for (int i = 0; i < this.waypoints.length; i++)
        {
            TrcWaypoint waypoint = new TrcWaypoint(this.waypoints[i]);
            // If already in degree mode, don't convert again.
            waypoint.heading = degrees ? waypoint.heading : Math.toDegrees(waypoint.heading);
            waypoints[i] = waypoint;
        }
        return new TrcPath(true, waypoints);
    }

    /**
     * Create a new path identical to this one, except the heading values are in radians. If this path's headings are
     * already in radians, just return deep copy.
     *
     * @return A TrcPath object which the same waypoints except all heading values are now in radians, if they weren't before.
     */
    public TrcPath toRadians()
    {
        TrcWaypoint[] waypoints = new TrcWaypoint[this.waypoints.length];
        for (int i = 0; i < this.waypoints.length; i++)
        {
            TrcWaypoint waypoint = new TrcWaypoint(this.waypoints[i]);
            // If already in degree mode, don't convert again.
            waypoint.heading = degrees ? Math.toRadians(waypoint.heading) : waypoint.heading;
            waypoints[i] = waypoint;
        }
        return new TrcPath(true, waypoints);
    }

    /**
     * If not already in degrees, convert this path's heading values to degrees.
     */
    public void mapSelfToDegrees()
    {
        if (!degrees)
        {
            degrees = true;
            for (TrcWaypoint waypoint : waypoints)
            {
                waypoint.heading = Math.toDegrees(waypoint.heading);
            }
        }
    }

    /**
     * If not already in radians, convert this path's heading values to radians.
     */
    public void mapSelfToRadians()
    {
        if (degrees)
        {
            degrees = false;
            for (TrcWaypoint waypoint : waypoints)
            {
                waypoint.heading = Math.toRadians(waypoint.heading);
            }
        }
    }

    /**
     * Get the estimated duration of the entire path.
     *
     * @return The estimated time duration, in the same units as <code>timeStep</code>.
     */
    public double getPathDuration()
    {
        double duration = 0;
        for (TrcWaypoint waypoint : waypoints)
        {
            duration += waypoint.timeStep;
        }
        return duration;
    }

    /**
     * Get the curved length of the entire path.
     *
     * @return The curved length of the entire path, in the same units as <code>x</code> and <code>y</code>.
     */
    public double getArcLength()
    {
        double length = 0;
        TrcWaypoint waypoint = waypoints[0];
        for (int i = 1; i < waypoints.length; i++)
        {
            length += waypoint.distanceTo(waypoints[i]);
            waypoint = waypoints[i];
        }
        return length;
    }

    /**
     * Use velocity and position data to infer the timesteps of the waypoint.
     */
    public void inferTimeSteps()
    {
        for (int i = 0; i < waypoints.length - 1; i++)
        {
            TrcWaypoint point = waypoints[i];
            TrcWaypoint next = waypoints[i + 1];
            double displacement = point.distanceTo(next);
            // Area of trapezoid: (v1+v2)/2 * t = d
            // t = d / ((v1+v2)/2)
            point.timeStep = displacement / TrcUtil.average(point.velocity, next.velocity);
        }
        // Assume last waypoint has the same timestep as second to last
        waypoints[waypoints.length - 1].timeStep = waypoints[waypoints.length - 2].timeStep;
    }
}
