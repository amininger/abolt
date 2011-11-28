package abolt.util;

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
            assert (keyValuePair.length > 1);
            if (keyValuePair[0].equals(tokenKey))
                return keyValuePair[1];
        }

        return null;    // No token found
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
