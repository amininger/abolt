package abolt.util;

import java.util.*;

public class SimUtil
{
    /** Give a string of key=token pairs, extract the token
     *  value corresponding to the given key
     **/
    public static String getTokenValue(String params, String tokenKey)
    {
        String[] tokens = params.split(",");
        for (String token: tokens) {
            String[] keyValuePair = token.split("=");
            if (keyValuePair.length < 2)
                continue;
            if (keyValuePair[0].equals(tokenKey))
                return keyValuePair[1];
        }

        return null;    // No token found
    }

    public static ArrayList<String> getPossibleValues(String[] pairs, String key)
    {
        ArrayList<String> values = new ArrayList<String>();
        for (String pair: pairs) {
            String[] keyValuePair = pair.split("=");
            if (keyValuePair[0].equals(key))
                values.add(keyValuePair[1]);
        }

        return values;
    }

    public static String nextValue(ArrayList<String> values, String value)
    {
        for (int i = 0; i < values.size(); i++) {
            if (values.get(i).equals(value))
                return values.get((i+1)%values.size());
        }
        return null;
    }

    static int id = 0;
    public static int nextID()
    {
        return id++;
    }

    static public void main(String[] args)
    {
        System.out.printf("%d %d %d\n", nextID(), nextID(), nextID());
    }
}
