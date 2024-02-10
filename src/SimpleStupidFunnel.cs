using Godot;
using Lanboost.PathFinding.Graph;
using System;
using System.Collections.Generic;

public class SSFState
{
    public Vector2 funnel;
    public Vector2 left;
    public Vector2 right;
    public Vector2 funnelSide;
    public int mode;


    public int leftIndex;
    public int rightIndex;

    public List<Vector2> pointList;

    public SSFState(Vector2 funnel, Vector2 left, Vector2 right, Vector2 funnelSide, int mode, int leftIndex, int rightIndex, List<Vector2> pointList)
    {
        this.funnel = funnel;
        this.left = left;
        this.right = right;
        this.funnelSide = funnelSide;
        this.mode = mode;
        this.leftIndex = leftIndex;
        this.rightIndex = rightIndex;
        this.pointList = pointList;
    }
}

public class SimpleStupidFunnel
{

    public static List<Vector2> Run(Vector2I start, Vector2I end, List<NavMeshEdge> edges)
    {
        List<Vector2> pointList = new List<Vector2>();
        (var leftPoints, var rightPoints) = CreateFunnelPoints(start, end, edges);

        Vector2 funnel = start;
        var left = leftPoints[0]-funnel;
        var right = rightPoints[0]-funnel;

        int rightIndex = 0;
        int leftIndex = 0;

        while(true)
        {
            if(rightIndex < leftIndex)
            {
                if (rightIndex + 1 >= rightPoints.Count)
                {
                    return pointList;
                }
                StepFunnel(ref funnel, ref right, ref left, ref rightIndex, ref leftIndex, rightPoints, leftPoints, 1, pointList);
            }
            else
            {
                if (leftIndex + 1 >= leftPoints.Count)
                {
                    return pointList;
                }
                StepFunnel(ref funnel, ref left, ref right, ref leftIndex, ref rightIndex, leftPoints, rightPoints, -1, pointList);
            }

            
        }
    }

    protected static void StepFunnel(ref Vector2 funnel, 
        ref Vector2 side,
        ref Vector2 otherSide,
        ref int index,
        ref int otherIndex,
        List<Vector2> toStep, 
        List<Vector2> other, 
        int negativeOperatior,
        List<Vector2> result
    )
    {
        var newSide = toStep[index + 1] - funnel;

        var value = newSide.Cross(side)* negativeOperatior;


        if (value < 0)
        {
            funnel = toStep[index];
            result.Add(funnel);
            side = toStep[index + 1] - funnel;
            otherSide = other[otherIndex] - funnel;
        }
        else
        {
            side = newSide;
            index++;
        }
    }

    public static (List<Vector2>, List<Vector2>) CreateFunnelPoints(Vector2 start, Vector2 end, List<NavMeshEdge> edges)
    {
        // We go from square rects, so we need to calculate which points will be left / right of the funnel.
        // One can thing of this as triangulating the edges.
        // And we need to check that the triangulation is faceing "up" with cross product

        List<Vector2> leftPoints = new List<Vector2>();
        List<Vector2> rightPoints = new List<Vector2>();

        var currentref = start;

        foreach(var edge in edges)
        {
            Vector2 left = new Vector2();
            Vector2 right = new Vector2();
            GetEdgeLeftRight(ref currentref, edge.edge, ref left, ref right);

            leftPoints.Add(left);
            rightPoints.Add(right);

            currentref = left;
        }

        leftPoints.Add(end);
        rightPoints.Add(end);

        return (leftPoints, rightPoints);
    }

    protected static void GetEdgeLeftRight(ref Vector2 reference, NavMeshRect rect, ref Vector2 left, ref Vector2 right)
    {
        if(rect.width == 0)
        {
            left.X = rect.sx;
            left.Y = rect.sy;
            right.X = rect.sx;
            right.Y = rect.sy+rect.height;
        }
        else
        {
            left.X = rect.sx;
            left.Y = rect.sy;
            right.X = rect.sx + rect.width;
            right.Y = rect.sy;
        }


        // Check which start edge is left, and which one is right
        // do this with cross product
        var leftref = left - reference;
        var rightref = right-reference;
        // If cross is negative, swap points
        if (leftref.Cross(rightref) < 0)
        {
            var t = left;
            left = right;
            right = t;
        }
    }

}
