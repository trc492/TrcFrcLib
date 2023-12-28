/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

import TrcCommonLib.trclib.TrcDbgTrace;

/**
 * This class implements the TrcDbgTrace.DbgLog interface which provides platform specific ways to print trace log
 * events to the debug log.
 */
public class FrcDbgLog implements TrcDbgTrace.DbgLog
{
    //
    // Implements TrcDbgTrace.DbgLog interface.
    //

    @Override
    public void msg(TrcDbgTrace.MsgLevel level, String msg)
    {
        String prefix;

        switch (level)
        {
            case FATAL:
                prefix = "_Fatal: ";
            break;

            case ERR:
                prefix = "_Err: ";
                break;
    
            case WARN:
                prefix = "_Warn: ";
                break;
    
            case INFO:
                prefix = "_Info: ";
                break;
    
            case VERBOSE:
                prefix = "_Verbose: ";
                break;
    
            default:
                prefix = "_Unk: ";
                break;
        }

        System.out.print(msg + prefix);
    }   //msg

}   //class FrcDbgLog
