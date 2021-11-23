/*
 * Copyright (c) 2020 Titan Robotics Club (http://www.titanrobotics.com)
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

package frclib;

import edu.wpi.first.wpilibj.util.Color;
import TrcCommonLib.trclib.TrcColor;
import TrcCommonLib.trclib.TrcUtil;

/**
 * This class implements an FRC specific color object. It extends TrcColor so it can be used by the rest of trclib.
 * It also includes an embedded WPILib Color object so it can be used by WPILib as well.
 */
public class FrcColor extends TrcColor
{
    //
    // Predefined colors.
    //
    public static final FrcColor BLACK = new FrcColor(0, 0, 0);
    public static final FrcColor FULL_RED = new FrcColor(255, 0, 0);
    public static final FrcColor FULL_GREEN = new FrcColor(0, 255, 0);
    public static final FrcColor FULL_BLUE = new FrcColor(0, 0, 255);
    public static final FrcColor FULL_YELLOW = new FrcColor(255, 255, 0);
    public static final FrcColor FULL_CYAN = new FrcColor(0, 255, 255);
    public static final FrcColor FULL_MAGENTA = new FrcColor(255, 0, 255);
    public static final FrcColor FULL_WHITE = new FrcColor(255, 255, 255);
    public static final FrcColor HALF_RED = new FrcColor(127, 0, 0);
    public static final FrcColor HALF_GREEN = new FrcColor(0, 127, 0);
    public static final FrcColor HALF_BLUE = new FrcColor(0, 0, 127);
    public static final FrcColor HALF_YELLOW = new FrcColor(127, 127, 0);
    public static final FrcColor HALF_CYAN = new FrcColor(0, 127, 127);
    public static final FrcColor HALF_MAGENTA = new FrcColor(127, 0, 127);
    public static final FrcColor HALF_WHITE = new FrcColor(127, 127, 127);

    public final Color color;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param red specifies the red value (0-255).
     * @param green specifies the green value (0-255).
     * @param blue specifies the blue value (0-255).
     */

    public FrcColor(int red, int green, int blue)
    {
        super(red, green, blue);
        color = new Color(
            TrcUtil.clipRange(red, 0, 255)/255.0,
            TrcUtil.clipRange(green, 0, 255)/255.0,
            TrcUtil.clipRange(blue, 0, 255)/255.0);
    }   //FrcColor

 }   //class FrcColor
