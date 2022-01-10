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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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

import TrcCommonLib.trclib.TrcHashMap;

/**
 * This class implements the FrcUserChoices object in which you can add named fields to the smart dashboard allowing
 * the drive team to select various choices for the match. The choices could be related to autonomous, subsystem
 * options, auto-assist or anything you want to alter how the robot program is run.
 */
public class FrcUserChoices
{
    private final FrcDashboard dashboard;
    private final TrcHashMap<String, Object> choiceMap;

    /**
     * Constructor: Create an instance of the object.
     */
    public FrcUserChoices()
    {
        dashboard = FrcDashboard.getInstance();
        choiceMap = new TrcHashMap<>();
    }   //FrcUserChoices

    /**
     * This method adds a choice menu to the dmart dashboard. This allows the drive team to pick options such as
     * autonomous strategies, or starting position.
     *
     * @param key specifies the dashboard key associated with the choice menu. This also determines which tab the
     *            choice menu will be placed on the smart dashboard.
     * @param choiceMenu specifies the choice menu to be added.
     */
    public void addChoiceMenu(String key, FrcChoiceMenu<?> choiceMenu)
    {
        choiceMap.add(key, choiceMenu);
    }   //addChoiceMenu

    /**
     * This method adds a string field to the smart dashboard.
     *
     * @param key specifies the dashboard key associated with the string field. This also determines which tab the
     *            string field will be placed on the smart dashboard.
     * @param value specifies the initial string value.
     */
    public void addString(String key, String value)
    {
        choiceMap.add(key, value);
        dashboard.refreshKey(key, value);
    }   //addString

    /**
     * This method adds a number field to the smart dashboard.
     *
     * @param key specifies the dashboard key associated with the number field. This also determines which tab the
     *            number field will be placed on the smart dashboard.
     * @param value specifies the initial number value.
     */
    public void addNumber(String key, double value)
    {
        choiceMap.add(key, value);
        dashboard.refreshKey(key, value);
    }   //addNumber

    /**
     * This method adds a boolean field to the smart dashboard.
     *
     * @param key specifies the dashboard key associated with the boolean field. This also determines which tab the
     *            boolean will be placed on the smart dashboard.
     * @param value specifies the initial boolean value.
     */
    public void addBoolean(String key, boolean value)
    {
        choiceMap.add(key, value);
        dashboard.refreshKey(key, value);
    }   //addBoolean

    /**
     * This method retrieves the current user choice object associated with the choice menu with the given name.
     *
     * @param key specifies the dashboard key associated with the choice menu.
     * @return current choice object from the assoicated choice menu.
     */
    public Object getUserChoice(String key)
    {
        Object obj = choiceMap.get(key);
    
        if (obj == null || !(obj instanceof FrcChoiceMenu))
        {
            throw new ClassCastException("\"" + key + "\"" + " is not a Choice Menu.");
        }

        return ((FrcChoiceMenu<?>) obj).getCurrentChoiceObject();
    }   //getUserChoice

    /**
     * This method retrieves the current string associated with the string field with the given name.
     *
     * @param key specifies the dashboard key associated with the string field.
     * @return current string value from the assoicated string field.
     */
    public String getUserString(String key)
    {
        Object obj = choiceMap.get(key);
    
        if (obj == null || !(obj instanceof String))
        {
            throw new ClassCastException("\"" + key + "\"" + " is not a String.");
        }

        return dashboard.getString(key, (String) obj);
    }   //getUserString

    /**
     * This method retrieves the current number associated with the number field with the given name.
     *
     * @param key specifies the dashboard key associated with the number field.
     * @return current number value from the assoicated number field.
     */
    public double getUserNumber(String key)
    {
        Object obj = choiceMap.get(key);
    
        if (obj == null || !(obj instanceof Double))
        {
            throw new ClassCastException("\"" + key + "\"" + " is not a Number.");
        }

        return dashboard.getNumber(key, (Double) obj);
    }   //getUserNumber

    /**
     * This method retrieves the current boolean associated with the boolean field with the given name.
     *
     * @param key specifies the dashboard key associated with the boolean field.
     * @return current boolean value from the assoicated boolean field.
     */
    public boolean getUserBoolean(String key)
    {
        Object obj = choiceMap.get(key);
    
        if (obj == null || !(obj instanceof Boolean))
        {
            throw new ClassCastException("\"" + key + "\"" + " is not a Boolean.");
        }

        return dashboard.getBoolean(key, (Boolean) obj);
    }   //getUserBoolean

}   //class FrcUserChoices
