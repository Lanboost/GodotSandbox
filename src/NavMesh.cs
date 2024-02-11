using Godot;
using Lanboost.PathFinding.Astar;
using Lanboost.PathFinding.Graph;
using System;
using System.Collections.Generic;

public class NavMesh
{


}


public class NavMeshEdge
{
    public NavMeshRect to;

    // Left / right here, is determined by which direction the edge is going
    // relative to the triangulation in Simple Stupid Funnel algorithm
    public Vector2 left;
    public Vector2 right;
    public float cost;

    public NavMeshEdge(NavMeshRect to, Vector2 left, Vector2 right, float cost)
    {
        this.to = to;
        this.left = left;
        this.right = right;
        this.cost = cost;
    }
}

public class NavMeshRect
{
    public int sx;
    public int sy;
    public int width;
    public int height;

    public List<NavMeshEdge> edges = new List<NavMeshEdge>();

    public NavMeshRect(int sx, int sy, int width, int height)
    {
        this.sx = sx;
        this.sy = sy;
        this.width = width;
        this.height = height;
    }

    public float ManhattanDistance(NavMeshRect other)
    {
        var midx1 = this.sx + this.width / 2;
        var midy1 = this.sy + this.height / 2;

        var midx2 = other.sx + other.width / 2;
        var midy2 = other.sy + other.height / 2;

        return Math.Abs(midx1 - midx2) + Math.Abs(midy1 - midy2);
    }

    public NavMeshEdge CreateEdge(NavMeshRect other)
    {
        // check box intersect
        if (this.sx <= other.sx + other.width && other.sx <= this.sx + this.width)
        {
            if (this.sy <= other.sy + other.height && other.sy <= this.sy + this.height)
            {
                Vector2 left, right;

                int sx, sy, ex, ey;
                // one side should line up with other
                // this is to the right going <-
                if (this.sx == other.sx + other.width)
                {
                    var ytop = Math.Max(this.sy, other.sy);
                    var ybottom = Math.Min(this.sy + this.height, other.sy + other.height);
                    left = new Vector2(this.sx, ybottom);
                    right = new Vector2(this.sx, ytop);
                }
                // left ->
                else if (this.sx + this.width == other.sx)
                {
                    var ytop = Math.Max(this.sy, other.sy);
                    var ybottom = Math.Min(this.sy + this.height, other.sy + other.height);
                    left = new Vector2(other.sx, ytop);
                    right = new Vector2(other.sx, ybottom);
                }
                // bottom ^
                else if (this.sy == other.sy + other.height)
                {
                    var xleft = Math.Max(this.sx, other.sx);
                    var xright = Math.Min(this.sx + this.width, other.sx + other.width);
                    left = new Vector2(xleft, this.sy);
                    right = new Vector2(xright, this.sy);

                }
                // top v
                else if (this.sy + this.height == other.sy)
                {
                    var xleft = Math.Max(this.sx, other.sx);
                    var xright = Math.Min(this.sx + this.width, other.sx + other.width);
                    left = new Vector2(xright, other.sy);
                    right = new Vector2(xleft, other.sy);

                }
                else
                {
                    throw new Exception("Wtf? Real overlap?");
                }

                return new NavMeshEdge(other, left, right, ManhattanDistance(other));
            }
        }
        return null;
    }
}


public class NavMeshGraph : IGraph<NavMeshRect, NavMeshEdge>
{
    AStar<NavMeshRect, NavMeshEdge> astar;

    public List<NavMeshRect> navMeshRects = new List<NavMeshRect>();

    public Dictionary<NavMeshRect, NavMeshEdge> extraEndEdges = new Dictionary<NavMeshRect, NavMeshEdge>();

    public NavMeshGraph(List<NavMeshRect> navMeshRects)
    {
        this.navMeshRects = navMeshRects;
        astar = new AStar<NavMeshRect, NavMeshEdge>(this, 100);
    }

    public NavMeshRect GetRectFromPosition(Vector2I position)
    {
        foreach (var rect in navMeshRects)
        {
            if (rect.sx <= position.X && position.X < rect.sx + rect.width)
            {
                if (rect.sy <= position.Y && position.Y < rect.sy + rect.height)
                {
                    return rect;
                }
            }
        }
        return null;
    }

    public void AddTemporaryStartEndNodes(NavMeshRect start, NavMeshRect end)
    {
        var s = GetRectFromPosition(new Vector2I(start.sx, start.sy));
        var e = GetRectFromPosition(new Vector2I(end.sx, end.sy));

        if (s == null || e == null)
        {
            //return null;
        }
        else
        {
            foreach(var se in s.edges)
            {
                var cost = start.ManhattanDistance(se.to);
                start.edges.Add(new NavMeshEdge(se.to, se.left, se.right, cost));
            }

            extraEndEdges.Clear();
            foreach (var ee in e.edges)
            {
                var cost = end.ManhattanDistance(ee.to);
                extraEndEdges.Add(ee.to,new NavMeshEdge(end, ee.right, ee.left, cost));
            }
        }
    }

    public int GetCost(NavMeshRect from, NavMeshRect to, NavMeshEdge link)
    {
        return (int)link.cost;
    }

    public IEnumerable<Edge<NavMeshRect, NavMeshEdge>> GetEdges(NavMeshRect node)
    {
        foreach (var edge in node.edges)
        {
            //var e = node != edge.left ? edge.left : edge.right;
            yield return new Edge<NavMeshRect, NavMeshEdge>(edge.to, edge);
        }
        if (extraEndEdges.ContainsKey(node))
        {
            yield return new Edge<NavMeshRect, NavMeshEdge>(extraEndEdges[node].to, extraEndEdges[node]);
        }
    }

    public int GetEstimation(NavMeshRect from, NavMeshRect to)
    {
        return (int)from.ManhattanDistance(to);
    }

    internal List<NavMeshEdge> FindPath(Vector2I start, Vector2I end)
    {
        

        var failure = this.astar.FindPath(new NavMeshRect(start.X, start.Y, 0, 0), new NavMeshRect(end.X, end.Y, 0,0));
        if (failure == null)
        {
            return this.astar.GetPathLinks();
        }
        else
        {
            GD.Print(failure);
        }
        return null;
    }
}
