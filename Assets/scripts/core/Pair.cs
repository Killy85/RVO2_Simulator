/*
 * Pair.cs
 * RVO2 Library C#
 * Added in order to replace the C++ Pair type
 */


public class Pair<T, U>
{
    public Pair()
    {
    }

    public Pair(T first, U second)
    {
        this.First = first;
        this.Second = second;
    }

    public T First { get; set; }
    public U Second { get; set; }

};