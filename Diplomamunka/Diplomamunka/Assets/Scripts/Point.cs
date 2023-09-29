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
    int outlineId;
    int lineId;

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

    public int OutlineId
    {
        get { return outlineId; }
        set { outlineId = value; }
    }

    public int LineId
    {
        get { return lineId; }
        set { lineId = value; }
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

    public Point(Vector3 _position, int _verticalIndex, int _horizontalIndex, int _id, int _outlineId, int _lineId)
    {
        position = _position;
        verticalIndex = _verticalIndex;
        horizontalIndex = _horizontalIndex;
        id = _id;
        outlineId = _outlineId;
        lineId = _lineId;
    }
}
