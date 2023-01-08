using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Point : MonoBehaviour
{
    Vector3 position;
    int row;
    int horizontalIndex;
    int verticalIndex;
    int id;

    public Vector3 Position
    {
        get { return position; }
        set { position = value; }
    }

    public int Id
    {
        get { return id; }
        set { id = value; }
    }

    public int VerticalIndex
    {
        get { return verticalIndex; }
        set { verticalIndex = value; }
    }

    public int HorizontalIndex
    {
        get { return horizontalIndex; }
        set { horizontalIndex = value; }
    }

    public Point(Vector3 _position, int _verticalIndex, int _horizontalIndex, int _id)
    {
        position = _position;
        verticalIndex = _verticalIndex;
        horizontalIndex = _horizontalIndex;
        id = _id;
    }
}
