/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
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

package TrcFrcLib.frclib;

import java.util.Date;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;

public class FrcMatchInfo
{
    private static FrcMatchInfo matchInfo = null;
    public final Date eventDate;
    public final String eventName;
    public final MatchType matchType;
    public final int matchNumber;
    public final int replayNumber;
    public final Alliance alliance;
    public final int location;
    public final String gameSpecificMessage;

    /**
     * This method returns the FRC match info. If none exists, it will create the FrcMatchInfo object.
     *
     * @return match info.
     */
    public static FrcMatchInfo getMatchInfo()
    {
        if (matchInfo == null || matchInfo.eventName == null)
        {
            matchInfo = new FrcMatchInfo();
        }

        return matchInfo;
    }   //getMatchInfo

    /**
     * Constructor: Create an instance of the object and initialize the fields accordingly.
     *
     * @param matchDate specifies the match date.
     */
    private FrcMatchInfo()
    {
        eventDate = new Date();
        if (DriverStation.isFMSAttached())
        {
            eventName = DriverStation.getEventName();
            matchType = DriverStation.getMatchType();
            matchNumber = DriverStation.getMatchNumber();
            replayNumber = DriverStation.getReplayNumber();
            alliance = DriverStation.getAlliance();
            location = DriverStation.getLocation();
            gameSpecificMessage = DriverStation.getGameSpecificMessage();
        }
        else
        {
            eventName = null;
            matchType = MatchType.None;
            matchNumber = 0;
            replayNumber = 0;
            alliance = Alliance.Invalid;
            location = 0;
            gameSpecificMessage = null;
        }
    }   //FrcMatchInfo

    /**
     * This method returns the string form of the match info.
     *
     * @return match info string.
     */
    @Override
    public String toString()
    {
        return "date=\"" + eventDate + "\"" +
               " event=\"" + eventName + "\"" +
               " type=\"" + matchType + "\"" +
               " match=" + matchNumber +
               " replay=" + replayNumber +
               " alliance=\"" + alliance + "\"" +
               " location=" + location +
               " gameMsg=\"" + gameSpecificMessage + "\"";
    }   //toString

}   //class FrcMatchInfo
